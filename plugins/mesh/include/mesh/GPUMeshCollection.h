/*
 * GPUMeshCollection.h
 *
 * Copyright (C) 2019 by Universitaet Stuttgart (VISUS).
 * All rights reserved.
 */

#ifndef GPU_MESH_DATA_STORAGE_H_INCLUDED
#define GPU_MESH_DATA_STORAGE_H_INCLUDED
#if (defined(_MSC_VER) && (_MSC_VER > 1000))
#    pragma once
#endif /* (defined(_MSC_VER) && (_MSC_VER > 1000)) */

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#define GLOWL_OPENGL_INCLUDE_GLAD
#include "glowl/Mesh.hpp"

namespace megamol {
namespace mesh {

class GPUMeshCollection {
public:
    template <typename T> using IteratorPair = std::pair<T, T>;

    struct BatchedMeshes {
        BatchedMeshes()
            : mesh(nullptr), vertices_allocated(0), vertices_used(0), indices_allocated(0), indices_used(0) {}

        BatchedMeshes(unsigned int vertices_allocated, unsigned int indices_allocated)
            : mesh(nullptr)
            , vertices_allocated(vertices_allocated)
            , vertices_used(0)
            , indices_allocated(indices_allocated)
            , indices_used(0) {}

        std::shared_ptr<glowl::Mesh> mesh;
        unsigned int vertices_allocated;
        unsigned int vertices_used;
        unsigned int indices_allocated;
        unsigned int indices_used;
    };

    struct SubMeshData {
        std::shared_ptr<BatchedMeshes> mesh;
        glowl::DrawElementsCommand sub_mesh_draw_command;
    };

    GPUMeshCollection() = default;
    ~GPUMeshCollection() = default;

    template <typename VertexBufferIterator, typename IndexBufferIterator>
    void addMesh(std::string const& identifier, std::vector<glowl::VertexLayout> const& vertex_descriptor,
        std::vector<IteratorPair<VertexBufferIterator>> const& vertex_buffers,
        IteratorPair<IndexBufferIterator> index_buffer, GLenum index_type, GLenum usage, GLenum primitive_type,
        bool store_seperate = false);

    void addMesh(std::string const& identifier, std::shared_ptr<glowl::Mesh> mesh, SubMeshData submesh);

    void deleteSubMesh(std::string const& identifier);

    void clear();

    SubMeshData const& getSubMesh(std::string const& identifier);

    std::vector<std::shared_ptr<BatchedMeshes>> const& getMeshes();

    std::unordered_map<std::string, SubMeshData> const& getSubMeshData();

private:
    std::vector<std::shared_ptr<BatchedMeshes>> m_batched_meshes;
    std::unordered_map<std::string, SubMeshData> m_sub_mesh_data;
};

template <typename VertexBufferIterator, typename IndexBufferIterator>
inline void GPUMeshCollection::addMesh(std::string const& identifier,
    std::vector<glowl::VertexLayout> const& vertex_descriptor,
    std::vector<IteratorPair<VertexBufferIterator>> const& vertex_buffers,
    IteratorPair<IndexBufferIterator> index_buffer, GLenum index_type, GLenum usage, GLenum primitive_type,
    bool store_seperate) {
    typedef typename std::iterator_traits<IndexBufferIterator>::value_type IndexBufferType;
    typedef typename std::iterator_traits<VertexBufferIterator>::value_type VertexBufferType;

    std::vector<size_t> vb_attrib_byte_sizes;
    for (auto& vertex_layout : vertex_descriptor) {
        vb_attrib_byte_sizes.push_back(vertex_layout.stride);
    }

    // get vertex buffer data pointers and byte sizes
    std::vector<GLvoid*> vb_data;
    std::vector<size_t> vb_byte_sizes;
    for (auto& vb : vertex_buffers) {
        vb_data.push_back(reinterpret_cast<GLvoid*>(&(*std::get<0>(vb))));
        vb_byte_sizes.push_back(sizeof(VertexBufferType) * std::distance(std::get<0>(vb), std::get<1>(vb)));
    }
    // compute overall byte size of index buffer
    size_t ib_byte_size =
        sizeof(VertexBufferType) * std::distance(std::get<0>(index_buffer), std::get<1>(index_buffer));

    // computer number of requested vertices and indices
    size_t req_vertex_cnt = vb_byte_sizes.front() / vb_attrib_byte_sizes.front();
    size_t req_index_cnt = ib_byte_size / glowl::computeByteSize(index_type);

    auto query = std::find_if(m_batched_meshes.begin(), m_batched_meshes.end(),
        [&vertex_descriptor, index_type, req_vertex_cnt, req_index_cnt](std::shared_ptr<BatchedMeshes> const& batched_meshes) {
            bool retval = true;
            for (int i = 0; i < vertex_descriptor.size(); ++i) {
                retval &= vertex_descriptor[i] == batched_meshes->mesh->getVertexLayouts()[i];
            }
            retval &= index_type == batched_meshes->mesh->getIndexType();

            auto ava_vertex_cnt = batched_meshes->vertices_allocated - batched_meshes->vertices_used;
            auto ava_index_cnt = batched_meshes->indices_allocated - batched_meshes->indices_used;
            retval &= ((req_vertex_cnt < ava_vertex_cnt) && (req_index_cnt < ava_index_cnt));
            return retval;
        });

    std::shared_ptr<BatchedMeshes> batched_mesh = nullptr;

    // if either no batch was found or mesh is requested to be stored seperated, create a new GPU mesh batch
    if (query == m_batched_meshes.end() || store_seperate) {
        size_t new_allocation_vertex_cnt = store_seperate ? req_vertex_cnt : std::max<size_t>(req_vertex_cnt, 800000);
        size_t new_allocation_index_cnt = store_seperate ? req_index_cnt : std::max<size_t>(req_index_cnt, 3200000);

        m_batched_meshes.push_back(std::make_shared<BatchedMeshes>(new_allocation_vertex_cnt, new_allocation_index_cnt));
        batched_mesh = m_batched_meshes.back();
        batched_mesh->vertices_used = 0;
        batched_mesh->indices_used = 0;

        std::vector<GLvoid*> alloc_data(vertex_buffers.size(), nullptr);
        std::vector<size_t> alloc_vb_byte_sizes;
        for (size_t attrib_byte_size : vb_attrib_byte_sizes) {
            alloc_vb_byte_sizes.push_back(attrib_byte_size * new_allocation_vertex_cnt);
        }

        batched_mesh->mesh = std::make_shared<glowl::Mesh>(alloc_data, alloc_vb_byte_sizes, nullptr,
            new_allocation_index_cnt * glowl::computeByteSize(index_type), vertex_descriptor, index_type, usage,
            primitive_type);
    } else {
        batched_mesh = (*query);
    }

    auto new_sub_mesh = SubMeshData();
    new_sub_mesh.mesh = batched_mesh;
    new_sub_mesh.sub_mesh_draw_command.first_idx = batched_mesh->indices_used;
    new_sub_mesh.sub_mesh_draw_command.base_vertex = batched_mesh->vertices_used;
    new_sub_mesh.sub_mesh_draw_command.cnt = req_index_cnt;
    new_sub_mesh.sub_mesh_draw_command.instance_cnt = 1;
    new_sub_mesh.sub_mesh_draw_command.base_instance = 0;
    m_sub_mesh_data.insert({identifier, new_sub_mesh});

    // upload data to GPU
    for (size_t i = 0; i < vb_data.size(); ++i) {
        // at this point, it should be guaranteed that it points at a mesh with matching vertex layout,
        // hence it's legal to multiply requested attrib byte sizes with vertex used count
        batched_mesh->mesh->bufferVertexSubData(
            i, vb_data[i], vb_byte_sizes[i], vb_attrib_byte_sizes[i] * (batched_mesh->vertices_used));
    }

    batched_mesh->mesh->bufferIndexSubData(reinterpret_cast<GLvoid*>(&*std::get<0>(index_buffer)), ib_byte_size,
        glowl::computeByteSize(index_type) * batched_mesh->indices_used);

    // updated vertices and indices used
    batched_mesh->vertices_used += req_vertex_cnt;
    batched_mesh->indices_used += req_index_cnt;
}

inline void GPUMeshCollection::addMesh(
    std::string const& identifier, std::shared_ptr<glowl::Mesh> mesh, SubMeshData submesh) {
    // TODO!!!
}

inline void GPUMeshCollection::deleteSubMesh(std::string const& identifier) {

    auto query = m_sub_mesh_data.find(identifier);

    if (query != m_sub_mesh_data.end()) {
        m_sub_mesh_data.erase(query);
    }
}

inline void GPUMeshCollection::clear() {
    m_batched_meshes.clear();
    m_sub_mesh_data.clear();
}

inline GPUMeshCollection::SubMeshData const& GPUMeshCollection::getSubMesh(std::string const& identifier) {

    auto query = m_sub_mesh_data.find(identifier);

    if (query != m_sub_mesh_data.end()) {
        return query->second;
    }

    return GPUMeshCollection::SubMeshData();
}

inline std::vector<std::shared_ptr<GPUMeshCollection::BatchedMeshes>> const& GPUMeshCollection::getMeshes() { return m_batched_meshes; }

inline std::unordered_map<std::string, GPUMeshCollection::SubMeshData> const& GPUMeshCollection::getSubMeshData() {
    return m_sub_mesh_data;
}

} // namespace mesh
} // namespace megamol

#endif // !GPU_MESH_DATA_STORAGE_H_INCLUDED
