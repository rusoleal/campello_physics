
#include "GLBScene.h"

#include <cmath>
#include <cstring>

namespace GLBScene {

// ── Low-level binary helpers ──────────────────────────────────────────────────

namespace {

template<typename T>
void write(std::vector<uint8_t>& buf, T value) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
    buf.insert(buf.end(), p, p + sizeof(T));
}

void writeF3(std::vector<uint8_t>& buf, float x, float y, float z) {
    write(buf, x); write(buf, y); write(buf, z);
}

void writeU16(std::vector<uint8_t>& buf, uint16_t v) { write(buf, v); }

void pad4(std::vector<uint8_t>& buf) {
    while (buf.size() % 4) buf.push_back(0x00);
}

// ── Mesh generators ───────────────────────────────────────────────────────────

struct MeshBuffers {
    std::vector<uint8_t> positions;   // float32 VEC3
    std::vector<uint8_t> normals;     // float32 VEC3
    std::vector<uint8_t> indices;     // uint16
    uint32_t vertCount = 0;
    uint32_t idxCount  = 0;
    float minPos[3] = { 0, 0, 0 };
    float maxPos[3] = { 0, 0, 0 };
};

// Unit UV sphere: radius 0.5, latSegs latitude rings, lonSegs longitude segments.
MeshBuffers makeSphere(int latSegs = 16, int lonSegs = 16) {
    MeshBuffers m;
    const float r = 0.5f;
    const float pi = 3.14159265358979323846f;

    for (int lat = 0; lat <= latSegs; ++lat) {
        float theta = (float)lat / (float)latSegs * pi;
        float sinT = sinf(theta), cosT = cosf(theta);
        for (int lon = 0; lon <= lonSegs; ++lon) {
            float phi = (float)lon / (float)lonSegs * 2.0f * pi;
            float sinP = sinf(phi), cosP = cosf(phi);
            float nx = sinT * cosP, ny = cosT, nz = sinT * sinP;
            writeF3(m.positions, nx * r, ny * r, nz * r);
            writeF3(m.normals,   nx,     ny,     nz);
            ++m.vertCount;
        }
    }
    m.minPos[0] = m.minPos[1] = m.minPos[2] = -r;
    m.maxPos[0] = m.maxPos[1] = m.maxPos[2] =  r;

    for (int lat = 0; lat < latSegs; ++lat) {
        for (int lon = 0; lon < lonSegs; ++lon) {
            uint16_t a = (uint16_t)(lat * (lonSegs + 1) + lon);
            uint16_t b = (uint16_t)(a + lonSegs + 1);
            writeU16(m.indices, a);   writeU16(m.indices, b);     writeU16(m.indices, (uint16_t)(a + 1));
            writeU16(m.indices, b);   writeU16(m.indices, (uint16_t)(b + 1)); writeU16(m.indices, (uint16_t)(a + 1));
            m.idxCount += 6;
        }
    }
    return m;
}

// Unit box: 1×1×1 m, 24 unique vertices (4 per face with correct normals).
MeshBuffers makeBox() {
    MeshBuffers m;
    const float h = 0.5f;
    struct Face { float nx, ny, nz; float verts[4][3]; };
    static const Face faces[6] = {
        { 0,  0,  1, {{ -h,-h, h}, { h,-h, h}, { h, h, h}, {-h, h, h}} }, // +Z
        { 0,  0, -1, {{  h,-h,-h}, {-h,-h,-h}, {-h, h,-h}, { h, h,-h}} }, // -Z
        { 0,  1,  0, {{ -h, h,-h}, { h, h,-h}, { h, h, h}, {-h, h, h}} }, // +Y
        { 0, -1,  0, {{ -h,-h, h}, { h,-h, h}, { h,-h,-h}, {-h,-h,-h}} }, // -Y
        { 1,  0,  0, {{  h,-h, h}, { h,-h,-h}, { h, h,-h}, { h, h, h}} }, // +X
        {-1,  0,  0, {{ -h,-h,-h}, {-h,-h, h}, {-h, h, h}, {-h, h,-h}} }, // -X
    };
    for (int f = 0; f < 6; ++f) {
        for (int v = 0; v < 4; ++v) {
            writeF3(m.positions, faces[f].verts[v][0], faces[f].verts[v][1], faces[f].verts[v][2]);
            writeF3(m.normals,   faces[f].nx, faces[f].ny, faces[f].nz);
        }
        uint16_t b = (uint16_t)(f * 4);
        writeU16(m.indices, b);   writeU16(m.indices, (uint16_t)(b+1)); writeU16(m.indices, (uint16_t)(b+2));
        writeU16(m.indices, b);   writeU16(m.indices, (uint16_t)(b+2)); writeU16(m.indices, (uint16_t)(b+3));
        m.vertCount += 4;
        m.idxCount  += 6;
    }
    m.minPos[0] = m.minPos[1] = m.minPos[2] = -h;
    m.maxPos[0] = m.maxPos[1] = m.maxPos[2] =  h;
    return m;
}

// Floor plane: 10×10 m at y = 0.
MeshBuffers makeFloor(float halfSize = 5.0f) {
    MeshBuffers m;
    float h = halfSize;
    const float verts[4][3] = { {-h,0,-h},{h,0,-h},{h,0,h},{-h,0,h} };
    for (int i = 0; i < 4; ++i) {
        writeF3(m.positions, verts[i][0], verts[i][1], verts[i][2]);
        writeF3(m.normals, 0.0f, 1.0f, 0.0f);
    }
    writeU16(m.indices, 0); writeU16(m.indices, 1); writeU16(m.indices, 2);
    writeU16(m.indices, 0); writeU16(m.indices, 2); writeU16(m.indices, 3);
    m.vertCount = 4; m.idxCount = 6;
    m.minPos[0] = m.minPos[2] = -h; m.maxPos[0] = m.maxPos[2] = h;
    m.minPos[1] = m.maxPos[1] = 0.0f;
    return m;
}

// ── GLB assembly ──────────────────────────────────────────────────────────────

std::string buildJSON(
    const std::vector<MeshBuffers>& meshes,
    const std::vector<uint32_t>& binOffsets,   // byte offsets into BIN chunk
    int numBodyNodes)
{
    // Build the GLTF JSON manually.
    // Accessor layout per mesh: [positions, normals, indices]
    // Accessor indices: mesh i → accessors [3i, 3i+1, 3i+2]
    // BufferView indices: same (1:1 with accessors)

    auto fmtF3 = [](float v) -> std::string {
        char buf[32]; snprintf(buf, sizeof(buf), "%.6f", (double)v); return buf;
    };

    std::string json = "{\n";
    json += "\"asset\":{\"version\":\"2.0\",\"generator\":\"campello_physics examples\"},\n";
    json += "\"scene\":0,\n";

    // Scene: floor node (0) + body nodes (1 … numBodyNodes)
    json += "\"scenes\":[{\"nodes\":[";
    for (int i = 0; i <= numBodyNodes; ++i) {
        if (i) json += ",";
        json += std::to_string(i);
    }
    json += "]}],\n";

    // Nodes: node 0 = floor, nodes 1..N = body slots (invisible initially, scale=0)
    json += "\"nodes\":[\n";
    // Floor node (mesh index 2)
    json += "{\"name\":\"floor\",\"mesh\":2,\"translation\":[0,0,0]}";
    for (int i = 0; i < numBodyNodes; ++i) {
        json += ",\n{\"name\":\"body" + std::to_string(i) +
                "\",\"mesh\":0,\"translation\":[0,-1000,0],\"scale\":[0,0,0]}";
    }
    json += "\n],\n";

    // Materials: grey for floor, teal for dynamic bodies
    json += "\"materials\":[\n";
    json += "{\"name\":\"body\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.2,0.6,0.9,1.0],\"metallicFactor\":0.0,\"roughnessFactor\":0.5}},\n";
    json += "{\"name\":\"floor\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.55,0.55,0.55,1.0],\"metallicFactor\":0.0,\"roughnessFactor\":0.9}}\n";
    json += "],\n";

    // Meshes: sphere (mat 0), box (mat 0), floor plane (mat 1)
    const char* meshNames[3] = {"sphere","box","floor"};
    int matIdx[3] = {0, 0, 1};
    json += "\"meshes\":[\n";
    for (int mi = 0; mi < 3; ++mi) {
        if (mi) json += ",\n";
        int ai = mi * 3;
        json += "{\"name\":\"" + std::string(meshNames[mi]) + "\",\"primitives\":[{";
        json += "\"attributes\":{\"POSITION\":" + std::to_string(ai) +
                ",\"NORMAL\":" + std::to_string(ai + 1) + "},";
        json += "\"indices\":" + std::to_string(ai + 2) + ",";
        json += "\"material\":" + std::to_string(matIdx[mi]) + "}]}";
    }
    json += "\n],\n";

    // Accessors and bufferViews
    json += "\"accessors\":[\n";
    json += "\"bufferViews\":[\n";
    std::string accessors, bufferViews;
    for (int mi = 0; mi < 3; ++mi) {
        const auto& m = meshes[mi];
        uint32_t base = binOffsets[mi];
        uint32_t posBytes  = (uint32_t)m.positions.size();
        uint32_t normBytes = (uint32_t)m.normals.size();
        uint32_t idxBytes  = (uint32_t)m.indices.size();

        auto fmtAcc3 = [&](int bvIdx, uint32_t count,
                           float mn0, float mn1, float mn2,
                           float mx0, float mx1, float mx2) {
            return "{\"bufferView\":" + std::to_string(bvIdx) +
                   ",\"componentType\":5126,\"count\":" + std::to_string(count) +
                   ",\"type\":\"VEC3\""
                   ",\"min\":[" + fmtF3(mn0) + "," + fmtF3(mn1) + "," + fmtF3(mn2) + "]"
                   ",\"max\":[" + fmtF3(mx0) + "," + fmtF3(mx1) + "," + fmtF3(mx2) + "]}";
        };
        auto fmtAccIdx = [&](int bvIdx, uint32_t count) {
            return "{\"bufferView\":" + std::to_string(bvIdx) +
                   ",\"componentType\":5123,\"count\":" + std::to_string(count) +
                   ",\"type\":\"SCALAR\"}";
        };
        auto fmtBV = [&](uint32_t byteOffset, uint32_t byteLen) {
            return "{\"buffer\":0,\"byteOffset\":" + std::to_string(byteOffset) +
                   ",\"byteLength\":" + std::to_string(byteLen) + "}";
        };

        int ai = mi * 3, bvi = mi * 3;
        if (ai) { accessors += ",\n"; bufferViews += ",\n"; }

        accessors += fmtAcc3(bvi,   m.vertCount,
            m.minPos[0],m.minPos[1],m.minPos[2],
            m.maxPos[0],m.maxPos[1],m.maxPos[2]) + ",\n";
        accessors += fmtAcc3(bvi+1, m.vertCount,
            -1,-1,-1, 1,1,1) + ",\n";
        accessors += fmtAccIdx(bvi+2, m.idxCount);

        bufferViews += fmtBV(base,             posBytes)  + ",\n";
        bufferViews += fmtBV(base + posBytes,  normBytes) + ",\n";
        bufferViews += fmtBV(base + posBytes + normBytes, idxBytes);
    }

    // Rebuild correctly: accessors and bufferViews were built above but JSON open already.
    // Rebuild the whole json from scratch more cleanly:
    json = "";
    json += "{\"asset\":{\"version\":\"2.0\",\"generator\":\"campello_physics examples\"},\n";
    json += "\"scene\":0,\n";
    json += "\"scenes\":[{\"nodes\":[";
    for (int i = 0; i <= numBodyNodes; ++i) {
        if (i) json += ",";
        json += std::to_string(i);
    }
    json += "]}],\n\"nodes\":[\n";
    json += "{\"name\":\"floor\",\"mesh\":2}";
    for (int i = 0; i < numBodyNodes; ++i) {
        json += ",\n{\"name\":\"body" + std::to_string(i) +
                "\",\"mesh\":0,\"translation\":[0,-1000,0],\"scale\":[0,0,0]}";
    }
    json += "\n],\n";
    json += "\"materials\":[\n";
    json += "{\"name\":\"body\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.2,0.6,0.9,1.0],\"metallicFactor\":0.0,\"roughnessFactor\":0.5}},\n";
    json += "{\"name\":\"floor\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.55,0.55,0.55,1.0],\"metallicFactor\":0.0,\"roughnessFactor\":0.9}}\n";
    json += "],\n";
    json += "\"meshes\":[\n";
    for (int mi = 0; mi < 3; ++mi) {
        if (mi) json += ",\n";
        int ai = mi * 3;
        json += "{\"name\":\"" + std::string(meshNames[mi]) + "\",\"primitives\":[{";
        json += "\"attributes\":{\"POSITION\":" + std::to_string(ai) +
                ",\"NORMAL\":" + std::to_string(ai + 1) + "},";
        json += "\"indices\":" + std::to_string(ai + 2) + ",";
        json += "\"material\":" + std::to_string(matIdx[mi]) + "}]}";
    }
    json += "\n],\n";
    json += "\"accessors\":[\n" + accessors + "\n],\n";
    json += "\"bufferViews\":[\n" + bufferViews + "\n],\n";

    // Total binary size
    uint32_t totalBin = 0;
    for (auto& m : meshes)
        totalBin += (uint32_t)(m.positions.size() + m.normals.size() + m.indices.size());

    json += "\"buffers\":[{\"byteLength\":" + std::to_string(totalBin) + "}]\n}";
    return json;
}

} // anonymous namespace

// ── GLBScene::build ───────────────────────────────────────────────────────────

std::vector<uint8_t> build() {
    // Generate meshes
    std::vector<MeshBuffers> meshes;
    meshes.push_back(makeSphere());
    meshes.push_back(makeBox());
    meshes.push_back(makeFloor());

    // Compute binary offsets
    std::vector<uint32_t> offsets;
    uint32_t off = 0;
    for (auto& m : meshes) {
        offsets.push_back(off);
        off += (uint32_t)(m.positions.size() + m.normals.size() + m.indices.size());
    }

    // Build JSON
    std::string jsonStr = buildJSON(meshes, offsets, kMaxBodies);
    // GLB requires JSON chunk to be 4-byte aligned with space padding
    while (jsonStr.size() % 4) jsonStr += ' ';

    // Build BIN chunk data
    std::vector<uint8_t> binData;
    for (auto& m : meshes) {
        binData.insert(binData.end(), m.positions.begin(), m.positions.end());
        binData.insert(binData.end(), m.normals.begin(),   m.normals.end());
        binData.insert(binData.end(), m.indices.begin(),   m.indices.end());
    }
    while (binData.size() % 4) binData.push_back(0);

    // Assemble GLB
    // Header: 12 bytes
    // JSON chunk header: 8 bytes + jsonStr.size()
    // BIN chunk header: 8 bytes + binData.size()
    uint32_t totalLen = 12
                      + 8 + (uint32_t)jsonStr.size()
                      + 8 + (uint32_t)binData.size();

    std::vector<uint8_t> glb;
    glb.reserve(totalLen);

    // GLB header
    write(glb, (uint32_t)0x46546C67); // magic "glTF"
    write(glb, (uint32_t)2);          // version
    write(glb, totalLen);

    // JSON chunk
    write(glb, (uint32_t)jsonStr.size());
    write(glb, (uint32_t)0x4E4F534A); // "JSON"
    glb.insert(glb.end(), (uint8_t*)jsonStr.data(),
                           (uint8_t*)jsonStr.data() + jsonStr.size());

    // BIN chunk
    write(glb, (uint32_t)binData.size());
    write(glb, (uint32_t)0x004E4942); // "BIN\0"
    glb.insert(glb.end(), binData.begin(), binData.end());

    return glb;
}

} // namespace GLBScene
