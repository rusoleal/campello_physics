
#pragma once

// GLBScene — procedural GLB scene generator for campello_physics examples.
//
// Generates a GLB file in memory containing:
//   Mesh 0: unit sphere  (radius = 0.5 m, UV sphere 16×16)
//   Mesh 1: unit box     (1×1×1 m)
//   Mesh 2: floor plane  (10×10 m, y = 0)
//
// The scene has one floor node (index 0) and kMaxBodies dynamic-body nodes
// (indices 1 … kMaxBodies). Callers update node transforms each frame from
// physics body data, then re-render.
//
// Usage:
//   auto glbData = GLBScene::build();
//   auto gltf    = gltf::GLTF::loadGLB(glbData.data(), glbData.size());
//   renderer->setAsset(gltf);
//   // each frame:
//   GLBScene::updateFloor(*gltf, floorPos, floorRot);
//   GLBScene::updateBody(*gltf, bodyIndex, pos, rot, meshType);
//   renderer->render();

#include <vector>
#include <cstdint>
#include <string>

namespace GLBScene {

// Maximum number of dynamic body nodes baked into the scene.
static constexpr int kMaxBodies = 64;

// Which mesh to assign to a body node.
enum class MeshType { Sphere = 0, Box = 1 };

// Build and return the raw GLB bytes.
std::vector<uint8_t> build();

} // namespace GLBScene
