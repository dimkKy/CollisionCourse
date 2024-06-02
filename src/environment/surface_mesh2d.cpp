#include "surface_mesh2d.h"

#include <godot_cpp\classes\array_mesh.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>
#include <execution>

#include "utils.h"

float SurfaceMesh2D::CurveFunction(float x)
{
    return 25.f * std::sinf(0.01f * x);
}

godot::Vector2 SurfaceMesh2D::CalcArraysDown(godot::PackedVector2Array& verts, godot::PackedVector2Array& uvs, float startX)
{
    //assumes arrays constrcuted from templates
    //TODO ADD RETURN VALUES

    float norm{ CurveFunction(startX) };
    startX += subSegmentWidth;
    float localMinHeight{ 0.f };

    for (int i{ 2 }; i < lastSubVertex; ++i) {
        //will it be optimized by a compiler?
        verts[i].y = CurveFunction(startX) - norm;
      
        if (verts[i].y > localMinHeight) {
            localMinHeight = verts[i].y;
        }
        startX += subSegmentWidth;
    }

    localMinHeight += minHeight;
    verts[0].y += localMinHeight;
    verts[lastSubVertex].y += localMinHeight;

    return verts[curvePointSubCount];
}

godot::Vector2 SurfaceMesh2D::CalcArraysUp(godot::PackedVector2Array& verts, godot::PackedVector2Array& uvs, float startX)
{
    //assumes arrays constrcuted from templates
    //TODO ADD RETURN VALUES

    float norm{ CurveFunction(startX) };
    startX += subSegmentWidth;
    float localMinHeight{ 0.f };

    for (int i{ 2 }; i < lastSubVertex; ++i) {
        //will it be optimized by a compiler?
        verts[i].y = CurveFunction(startX) - norm;

        if (verts[i].y < localMinHeight) {
            localMinHeight = verts[i].y;
        }
        startX += subSegmentWidth;
    }

    localMinHeight -= minHeight;
    verts[0].y -= localMinHeight;
    verts[lastSubVertex].y -= localMinHeight;

    return verts[curvePointSubCount];
}

godot::PackedInt32Array SurfaceMesh2D::CalcIndexesTemplate()
{
    godot::PackedInt32Array arr{};
    for (int i{ 1 }; i < midSubSegment; ++i) {
        arr.push_back(0);
        arr.push_back(i);
        arr.push_back(i + 1);
    }
    arr.push_back(0);
    arr.push_back(midSubSegment);
    arr.push_back(lastSubVertex);
    for (int i{ midSubSegment }; i < lastSubVertex - 1; ++i) {
        arr.push_back(i);
        arr.push_back(i + 1);
        arr.push_back(lastSubVertex);
    }
    return arr;
}

godot::PackedVector2Array SurfaceMesh2D::CalcVertsTemplate()
{
    //error here, неверная трактовка числа колво сегментов
    godot::PackedVector2Array arr{};
    float xCoord{ 0.f };
    arr.push_back({ xCoord, 0.f });
    for (int i{ 0 }; i < subSegmentCount; ++i) {
        arr.push_back({ xCoord, 0.f });
        xCoord += subSegmentWidth;
    }
    //xCoord += segmentWidth;
    arr.push_back({ xCoord, 0.f });
    arr.push_back({ xCoord, 0.f });
    return arr;
}

godot::PackedVector2Array SurfaceMesh2D::CalcUVsTemplate()
{
    //TODO
    return godot::PackedVector2Array();
}

const godot::PackedInt32Array& SurfaceMesh2D::GetIndexesTemplateRef()
{
    static auto indexesTemplate = CalcIndexesTemplate();
    return indexesTemplate;
}

const godot::PackedVector2Array& SurfaceMesh2D::GetVertsTemplateRef()
{
    static auto vertsTemplate = CalcVertsTemplate();
    return vertsTemplate;
}

const godot::PackedVector2Array& SurfaceMesh2D::GetUVsTemplateRef()
{
    static auto uvsTemplate = CalcUVsTemplate();
    return uvsTemplate;
}

void SurfaceMesh2D::RetreiveChildren(godot::Node* owner)&
{
    auto spawner = [owner, this](segmentPairType& t) { 
        t.first = Utils::NewChild<godot::CollisionPolygon2D>(*this, owner);
        t.second = Utils::NewChild<godot::MeshInstance2D>(*t.first, owner);
    };
    
    std::for_each(std::execution::unseq,
        segments.begin(), segments.end(), spawner);
}

void SurfaceMesh2D::ConfigureSegment(godot::Vector2& startPos, segmentPairType& pair, float startX)&
{
    pair.first->set_position(startPos);

    godot::PackedVector2Array verts{ GetVertsTemplateRef() };
    godot::PackedVector2Array uvs{ GetUVsTemplateRef() };
    godot::PackedInt32Array indexes{ GetIndexesTemplateRef() };

    startPos += CalcArraysDown(verts, uvs, startX);

    godot::Ref<godot::ArrayMesh> mesh;
    mesh.instantiate();

    godot::Array arr;
    arr.resize(godot::Mesh::ARRAY_MAX);
    arr[godot::Mesh::ARRAY_VERTEX] = verts;
    //arr[godot::Mesh::ARRAY_TEX_UV] = uvs;
    arr[godot::Mesh::ARRAY_INDEX] = indexes;   

    mesh->add_surface_from_arrays(godot::Mesh::PRIMITIVE_TRIANGLES, arr, {}, {}, godot::Mesh::ARRAY_FLAG_USE_2D_VERTICES);

    pair.first->set_polygon(verts); 
    pair.second->set_mesh(mesh);
}

void SurfaceMesh2D::ConfigureSegments(float startX)&
{
    godot::Vector2 startPos{ totalWidth * -0.5f, 0.f };
    for (auto& pair : segments) {
        ConfigureSegment(startPos, pair, startX);
        startX += segmentWidth;
    }
}

void SurfaceMesh2D::CalcTemplateArrays()
{
    //indexesTemplate = CalcIndexesTemplate();
    //vertsTemplate = CalcVertsTemplate();
    //uvsTemplate = CalcUVsTemplate();

    //TODO UVS
}

void SurfaceMesh2D::_ready()
{
    RetreiveChildren(get_owner());
    ConfigureSegments(get_global_position().x);

}

void SurfaceMesh2D::_bind_methods()
{
}
