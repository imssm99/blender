/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>
#include <optional>

#include "BKE_curves.hh"

#include "BLI_index_mask_ops.hh"
#include "BLI_math_vec_types.hh"

#include "gpencil_new_proposal.hh"
#include "testing/testing.h"

namespace blender::bke {

class GPLayerGroup : ::GPLayerGroup {
 public:
  GPLayerGroup()
  {
    this->children = nullptr;
    this->children_size = 0;
    this->layer_indices = nullptr;
    this->layer_indices_size = 0;
  }

  GPLayerGroup(const StringRefNull name) : GPLayerGroup()
  {
    BLI_assert(name.size() < 128);
    strcpy(this->name, name.c_str());
  }

  ~GPLayerGroup()
  {
    /* Recursivly free the children of this layer group first. */
    for (int i = 0; i < this->children_size; i++) {
      MEM_delete(&this->children[i]);
    }
    /* Then free its data. */
    MEM_SAFE_FREE(this->children);
    MEM_SAFE_FREE(this->layer_indices);
  }

  IndexMask layers_index_mask()
  {
    return {reinterpret_cast<int64_t>(this->layer_indices), this->layer_indices_size};
  }
};

class GPDataRuntime {
 public:
  /* mutable void *sbuffer */

  /**
   * Cache that maps the index of a layer to the index mask of the frames in that layer.
   */
  mutable Map<int, Vector<int64_t>> frame_index_masks_cache;
  mutable std::mutex frame_index_masks_cache_mutex;

  IndexMask get_cached_frame_index_mask(int layer_index)
  {
    return frame_index_masks_cache.lookup(layer_index).as_span();
  }
};

/**
 * A wrapper class around a single curve in GPFrame.strokes (CurvesGeometry). It holds the offset
 * of where to find the stroke in the frame and it's size.
 * This class is only meant to facilitate the handling of individual strokes.
 */
class GPStroke : NonCopyable, NonMovable {
 public:
  GPStroke(CurvesGeometry *geometry, int num_points, int offset)
      : geometry_(geometry), points_num_(num_points), offset_(offset){};

  ~GPStroke() = default;

  int points_num() const
  {
    return points_num_;
  }

  /**
   * Start index of this stroke in the points array of geometry_.
   */
  int points_offset() const
  {
    return offset_;
  }

  Span<float3> points_positions() const
  {
    return {geometry_->positions().begin() + offset_, points_num_};
  }

  MutableSpan<float3> points_positions_for_write() const
  {
    return {geometry_->positions_for_write().begin() + offset_, points_num_};
  }

 private:
  CurvesGeometry *geometry_ = nullptr;
  int points_num_ = 0;
  int offset_;
};

class GPFrame : public ::GPFrame {
 public:
  GPFrame() : GPFrame(-1, -1)
  {
  }

  GPFrame(int start_frame) : GPFrame(start_frame, -1)
  {
  }

  GPFrame(int start_frame, int end_frame)
  {
    this->start = start_frame;
    this->end = end_frame;
    this->strokes = nullptr;
  }

  GPFrame(const GPFrame &other) : GPFrame(other.start, other.end)
  {
    if (other.strokes != nullptr) {
      this->strokes_as_curves() = CurvesGeometry::wrap(*other.strokes);
    }
    this->layer_index = other.layer_index;
  }

  GPFrame &operator=(const GPFrame &other)
  {
    if (this != &other && other.strokes != nullptr) {
      this->strokes_as_curves() = CurvesGeometry::wrap(*other.strokes);
    }
    this->layer_index = other.layer_index;
    this->start = other.start;
    this->end = other.end;
    return *this;
  }

  GPFrame(GPFrame &&other) : GPFrame(other.start, other.end)
  {
    if (this != &other) {
      std::swap(this->strokes, other.strokes);
    }
    this->layer_index = other.layer_index;
  }

  GPFrame &operator=(GPFrame &&other)
  {
    if (this != &other) {
      std::swap(this->strokes, other.strokes);
    }
    this->layer_index = other.layer_index;
    this->start = other.start;
    this->end = other.end;
    return *this;
  }

  ~GPFrame()
  {
    MEM_delete(reinterpret_cast<CurvesGeometry *>(this->strokes));
    this->strokes = nullptr;
  }

  bool operator<(const GPFrame &other) const
  {
    if (this->start == other.start) {
      return this->layer_index < other.layer_index;
    }
    return this->start < other.start;
  }

  /* Assumes that elem.first is the layer index and elem.second is the frame start. */
  bool operator<(const std::pair<int, int> elem) const
  {
    if (this->start == elem.second) {
      return this->layer_index < elem.first;
    }
    return this->start < elem.second;
  }

  bool operator==(const GPFrame &other) const
  {
    return this->layer_index == other.layer_index && this->start == other.start;
  }

  CurvesGeometry &strokes_as_curves()
  {
    return CurvesGeometry::wrap(*this->strokes);
  }

  int strokes_num() const
  {
    if (this->strokes == nullptr) {
      return 0;
    }
    return this->strokes->curve_num;
  }

  GPStroke add_new_stroke(int new_points_num)
  {
    if (this->strokes == nullptr) {
      this->strokes = MEM_new<CurvesGeometry>(__func__);
    }
    CurvesGeometry &strokes = this->strokes_as_curves();
    int orig_last_offset = strokes.offsets().last();

    strokes.resize(strokes.points_num() + new_points_num, strokes.curves_num() + 1);
    strokes.offsets_for_write().last() = strokes.points_num();

    /* Use ploy type by default. */
    strokes.curve_types_for_write().last() = CURVE_TYPE_POLY;

    strokes.tag_topology_changed();
    return {reinterpret_cast<CurvesGeometry *>(this->strokes), new_points_num, orig_last_offset};
  }
};

class GPLayer : public ::GPLayer {
 public:
  GPLayer() : GPLayer("GP_Layer")
  {
  }

  GPLayer(const StringRefNull name)
  {
    strcpy(this->name, name.c_str());
  }

  ~GPLayer() = default;

  bool operator==(const GPLayer &other) const
  {
    return STREQ(this->name, other.name);
  }
};

class GPData : public ::GPData {
 public:
  GPData() : GPData(0, 0)
  {
  }

  GPData(const int layers_size, const int frame_size)
  {
    BLI_assert(layers_size >= 0);
    BLI_assert(frame_size >= 0);

    this->frames_size = frame_size;
    this->layers_size = layers_size;

    if (this->frames_size > 0) {
      this->frames_array = reinterpret_cast<::GPFrame *>(
          MEM_calloc_arrayN(this->frames_size, sizeof(::GPFrame), __func__));
      default_construct_n(reinterpret_cast<GPFrame *>(this->frames_array), this->frames_size);
    }
    else {
      this->frames_array = nullptr;
    }
    CustomData_reset(&this->frame_data);

    if (this->layers_size > 0) {
      this->layers_array = reinterpret_cast<::GPLayer *>(
          MEM_calloc_arrayN(this->layers_size, sizeof(::GPLayer), __func__));
      this->active_layer_index = 0;
    }
    else {
      this->layers_array = nullptr;
      this->active_layer_index = -1;
    }

    this->default_group = MEM_new<::GPLayerGroup>(__func__);

    this->runtime = MEM_new<GPDataRuntime>(__func__);
  }

  ~GPData()
  {
    /* Free frames and frame custom data. */
    destruct_n(reinterpret_cast<GPFrame *>(this->frames_array), this->frames_size);
    MEM_SAFE_FREE(this->frames_array);
    CustomData_free(&this->frame_data, this->frames_size);

    /* Free layer and layer groups. */
    destruct_n(reinterpret_cast<GPLayer *>(this->layers_array), this->layers_size);
    MEM_SAFE_FREE(this->layers_array);
    MEM_delete(reinterpret_cast<GPLayerGroup *>(this->default_group));
    this->default_group = nullptr;

    MEM_delete(this->runtime);
    this->runtime = nullptr;
  }

  Span<GPFrame> frames() const
  {
    return {(const GPFrame *)this->frames_array, this->frames_size};
  }

  const GPFrame &frames(int index) const
  {
    return this->frames()[index];
  }

  MutableSpan<GPFrame> frames_for_write()
  {
    return {(GPFrame *)this->frames_array, this->frames_size};
  }

  GPFrame &frames_for_write(int index)
  {
    return this->frames_for_write()[index];
  }

  IndexMask frames_on_layer(int layer_index) const
  {
    if (layer_index < 0 || layer_index > this->layers_size) {
      return IndexMask();
    }

    /* If the indices are cached for this layer, use the cache. */
    if (this->runtime->frame_index_masks_cache.contains(layer_index)) {
      return this->runtime->get_cached_frame_index_mask(layer_index);
    }

    /* A double checked lock. */
    std::scoped_lock{this->runtime->frame_index_masks_cache_mutex};
    if (this->runtime->frame_index_masks_cache.contains(layer_index)) {
      return this->runtime->get_cached_frame_index_mask(layer_index);
    }

    Vector<int64_t> indices;
    const IndexMask mask = index_mask_ops::find_indices_based_on_predicate(
        IndexMask(this->frames_size), 1024, indices, [&](const int index) {
          return this->frames()[index].layer_index == layer_index;
        });

    /* Cache the resulting index mask. */
    this->runtime->frame_index_masks_cache.add(layer_index, std::move(indices));
    return mask;
  }

  IndexMask frames_on_layer(GPLayer &layer) const
  {
    int index = this->layers().first_index_try(layer);
    if (index == -1) {
      return IndexMask();
    }
    return frames_on_layer(index);
  }

  IndexMask frames_on_active_layer() const
  {
    return frames_on_layer(this->active_layer_index);
  }

  Span<GPLayer> layers() const
  {
    return {(const GPLayer *)this->layers_array, this->layers_size};
  }

  const GPLayer &layers(int index) const
  {
    return layers()[index];
  }

  MutableSpan<GPLayer> layers_for_write()
  {
    return {(GPLayer *)this->layers_array, this->layers_size};
  }

  GPLayer &layers_for_write(int index)
  {
    return layers_for_write()[index];
  }

  int add_layer(StringRefNull name)
  {
    /* Ensure that the layer array has enough space. */
    if (!ensure_layers_array_has_size_at_least(this->layers_size + 1)) {
      return -1;
    }

    GPLayer new_layer(name);
    /* Move new_layer to the end in the array. */
    this->layers_for_write().last() = std::move(new_layer);
    return this->layers_size - 1;
  }

  int add_frame_on_layer(int layer_index, int frame_start)
  {
    /* TODO: Check for collisions. */

    if (!ensure_frames_array_has_size_at_least(this->frames_size + 1)) {
      return -1;
    }

    GPFrame new_frame(frame_start);
    new_frame.layer_index = layer_index;
    this->frames_for_write().last() = std::move(new_frame);

    /* Sort frame array. */
    update_frames_array();

    auto it = std::lower_bound(this->frames().begin(),
                               this->frames().end(),
                               std::pair<int, int>(layer_index, new_frame.start));
    if (it == this->frames().end() || it->start != new_frame.start) {
      return -1;
    }
    return std::distance(this->frames().begin(), it);
  }

  int add_frame_on_layer(GPLayer &layer, int frame_start)
  {
    int index = this->layers().first_index_try(layer);
    if (index == -1) {
      return -1;
    }
    return add_frame_on_layer(index, frame_start);
  }

  int strokes_num() const
  {
    /* TODO: could be done with parallel_for */
    int count = 0;
    for (const GPFrame &gpf : this->frames()) {
      count += gpf.strokes_num();
    }
    return count;
  }

  void set_active_layer(int layer_index)
  {
    if (layer_index < 0 || layer_index >= this->layers_size) {
      return;
    }
    this->active_layer_index = layer_index;
  }

 private:
  const bool ensure_layers_array_has_size_at_least(int64_t size)
  {
    if (this->layers_size > size) {
      return true;
    }

    int old_size = this->layers_size;
    this->layers_size = size;

    ::GPLayer *new_array = reinterpret_cast<::GPLayer *>(
        MEM_calloc_arrayN(this->layers_size, sizeof(::GPLayer), __func__));
    if (new_array == nullptr) {
      return false;
    }

    if (this->layers_array != nullptr) {
      /* Since the layers have default move constructors, we just use memcpy here. */
      memcpy(new_array, this->layers_array, old_size * sizeof(::GPLayer));
      MEM_SAFE_FREE(this->layers_array);
    }
    this->layers_array = new_array;

    return true;
  }

  const bool ensure_frames_array_has_size_at_least(int64_t size)
  {
    if (this->frames_size > size) {
      return true;
    }

    int old_size = this->frames_size;
    this->frames_size = size;

    ::GPFrame *new_array = reinterpret_cast<::GPFrame *>(
        MEM_calloc_arrayN(this->frames_size, sizeof(::GPFrame), __func__));
    if (new_array == nullptr) {
      return false;
    }

    if (this->frames_array != nullptr) {
      uninitialized_relocate_n(reinterpret_cast<GPFrame *>(this->frames_array),
                               old_size,
                               reinterpret_cast<GPFrame *>(new_array));
      MEM_SAFE_FREE(this->frames_array);
      this->frames_array = new_array;
    }
    else {
      this->frames_array = new_array;
      default_construct_n(reinterpret_cast<GPFrame *>(this->frames_array), this->frames_size);
    }
    return true;
  }

  void update_frames_array()
  {
    /* Make sure frames are ordered chronologically and by layer order. */
    std::sort(this->frames_for_write().begin(), this->frames_for_write().end());

    /* Clear the cached indices since they are probably no longer valid. */
    this->runtime->frame_index_masks_cache.clear();
  }
};

}  // namespace blender::bke

namespace blender::bke::gpencil::tests {

TEST(gpencil_proposal, EmptyGPData)
{
  GPData data;
  EXPECT_EQ(data.layers_size, 0);
  EXPECT_EQ(data.frames_size, 0);
}

TEST(gpencil_proposal, OneLayer)
{
  GPData data(1, 0);
  EXPECT_EQ(data.layers_size, 1);
  EXPECT_EQ(data.frames_size, 0);
}

TEST(gpencil_proposal, LayerName)
{
  GPLayer layer1;
  EXPECT_STREQ(layer1.name, "GP_Layer");

  GPLayer layer2("FooLayer");
  EXPECT_STREQ(layer2.name, "FooLayer");
}

TEST(gpencil_proposal, AddOneLayer)
{
  GPData data;

  const int layer_index = data.add_layer("FooLayer");
  EXPECT_EQ(data.layers_size, 1);
  EXPECT_STREQ(data.layers(layer_index).name, "FooLayer");
}

TEST(gpencil_proposal, AddLayers)
{
  GPData data;
  StringRefNull layer_names[3] = {"TestLayer1", "TestLayer2", "TestLayer3"};

  for (int i : IndexRange(3)) {
    data.add_layer(layer_names[i]);
  }
  EXPECT_EQ(data.layers_size, 3);

  for (int i : IndexRange(3)) {
    EXPECT_STREQ(data.layers(i).name, layer_names[i].c_str());
  }
}

TEST(gpencil_proposal, ChangeLayerName)
{
  GPData data;

  const int layer_index = data.add_layer("FooLayer");
  EXPECT_EQ(data.layers_size, 1);
  EXPECT_STREQ(data.layers(layer_index).name, "FooLayer");

  strcpy(data.layers_for_write(layer_index).name, "BarLayer");

  EXPECT_EQ(data.layers_size, 1);
  EXPECT_STREQ(data.layers(layer_index).name, "BarLayer");
}

TEST(gpencil_proposal, AddFrameToLayer)
{
  GPData data;

  data.add_layer("TestLayer1");
  const int layer2_index = data.add_layer("TestLayer2");

  const int frame_index = data.add_frame_on_layer(layer2_index, 0);
  EXPECT_NE(frame_index, -1);

  EXPECT_EQ(data.frames_size, 1);
  EXPECT_EQ(data.frames().last().layer_index, 1);
  EXPECT_EQ(data.frames(frame_index).layer_index, 1);

  data.frames_for_write(frame_index).start = 20;
  EXPECT_EQ(data.frames(frame_index).start, 20);
}

TEST(gpencil_proposal, CheckFramesSorted1)
{
  GPData data;

  const int frame_numbers1[5] = {10, 5, 6, 1, 3};
  const int frame_numbers_sorted1[5] = {1, 3, 5, 6, 10};

  int layer1_index = data.add_layer("TestLayer1");
  for (int i : IndexRange(5)) {
    const int frame_index = data.add_frame_on_layer(layer1_index, frame_numbers1[i]);
    EXPECT_NE(frame_index, -1);
    EXPECT_EQ(data.frames(frame_index).start, frame_numbers1[i]);
  }

  for (const int i : data.frames().index_range()) {
    EXPECT_EQ(data.frames(i).start, frame_numbers_sorted1[i]);
  }
}

TEST(gpencil_proposal, CheckFramesSorted2)
{
  GPData data;

  const int frame_numbers_layer1[5] = {10, 5, 6, 1, 3};
  const int frame_numbers_layer2[5] = {8, 5, 7, 1, 4};
  const int frame_numbers_sorted2[10][2] = {
      {0, 1}, {1, 1}, {0, 3}, {1, 4}, {0, 5}, {1, 5}, {0, 6}, {1, 7}, {1, 8}, {0, 10}};

  const int layer1_index = data.add_layer("TestLayer1");
  const int layer2_index = data.add_layer("TestLayer2");
  for (int i : IndexRange(5)) {
    data.add_frame_on_layer(layer1_index, frame_numbers_layer1[i]);
    data.add_frame_on_layer(layer2_index, frame_numbers_layer2[i]);
  }

  for (const int i : data.frames().index_range()) {
    EXPECT_EQ(data.frames(i).layer_index, frame_numbers_sorted2[i][0]);
    EXPECT_EQ(data.frames(i).start, frame_numbers_sorted2[i][1]);
  }
}

TEST(gpencil_proposal, IterateOverFramesOnLayer)
{
  GPData data;

  const int frame_numbers_layer1[5] = {10, 5, 6, 1, 3};
  const int frame_numbers_layer2[5] = {8, 5, 7, 1, 4};

  const int frame_numbers_sorted1[5] = {1, 3, 5, 6, 10};
  const int frame_numbers_sorted2[5] = {1, 4, 5, 7, 8};

  const int layer1_index = data.add_layer("TestLayer1");
  const int layer2_index = data.add_layer("TestLayer2");
  for (int i : IndexRange(5)) {
    data.add_frame_on_layer(layer1_index, frame_numbers_layer1[i]);
    data.add_frame_on_layer(layer2_index, frame_numbers_layer2[i]);
  }

  IndexMask indices_frames_layer1 = data.frames_on_layer(layer1_index);
  EXPECT_TRUE(data.runtime->frame_index_masks_cache.contains(layer1_index));
  for (const int i : indices_frames_layer1.index_range()) {
    EXPECT_EQ(data.frames(indices_frames_layer1[i]).start, frame_numbers_sorted1[i]);
  }

  IndexMask indices_frames_layer2 = data.frames_on_layer(layer2_index);
  EXPECT_TRUE(data.runtime->frame_index_masks_cache.contains(layer2_index));
  for (const int i : indices_frames_layer2.index_range()) {
    EXPECT_EQ(data.frames(indices_frames_layer2[i]).start, frame_numbers_sorted2[i]);
  }
}

TEST(gpencil_proposal, AddSingleStroke)
{
  GPData data;
  const int layer1_index = data.add_layer("TestLayer1");

  const int frame_index = data.add_frame_on_layer(layer1_index, 0);
  EXPECT_NE(frame_index, -1);
  GPStroke stroke = data.frames_for_write(frame_index).add_new_stroke(100);

  EXPECT_EQ(data.strokes_num(), 1);
  EXPECT_EQ(data.frames(frame_index).strokes_num(), 1);
  EXPECT_EQ(stroke.points_num(), 100);
}

TEST(gpencil_proposal, ChangeStrokePoints)
{
  GPData data;
  const int layer1_index = data.add_layer("TestLayer1");

  static const Array<float3> test_positions{{
      {1.0f, 2.0f, 3.0f},
      {4.0f, 5.0f, 6.0f},
      {7.0f, 8.0f, 9.0f},
  }};

  const int frame_index = data.add_frame_on_layer(layer1_index, 0);
  EXPECT_NE(frame_index, -1);
  GPStroke stroke = data.frames_for_write(frame_index).add_new_stroke(test_positions.size());

  for (const int i : stroke.points_positions_for_write().index_range()) {
    stroke.points_positions_for_write()[i] = test_positions[i];
  }

  for (const int i : stroke.points_positions().index_range()) {
    EXPECT_V3_NEAR(stroke.points_positions()[i], test_positions[i], 1e-5f);
  }
}

}  // namespace blender::bke::gpencil::tests