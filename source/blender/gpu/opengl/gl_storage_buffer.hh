/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "MEM_guardedalloc.h"

#include "gpu_storage_buffer_private.hh"

#include "glew-mx.h"

namespace blender {
namespace gpu {

/**
 * Implementation of Storage Buffers using OpenGL.
 */
class GLStorageBuf : public StorageBuf {
 private:
  /** Slot to which this UBO is currently bound. -1 if not bound. */
  int slot_ = -1;
  /** OpenGL Object handle. */
  GLuint ssbo_id_ = 0;
  /** Usage type. */
  GPUUsageType usage_;

 public:
  GLStorageBuf(size_t size, GPUUsageType usage, const char *name);
  ~GLStorageBuf();

  void update(const void *data) override;
  void bind(int slot) override;
  void unbind() override;

 private:
  void init();

  MEM_CXX_CLASS_ALLOC_FUNCS("GLStorageBuf");
};

}  // namespace gpu
}  // namespace blender