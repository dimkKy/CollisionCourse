// by Dmitry Kolontay

#pragma once

#include <godot_cpp\classes\mesh_instance2d.hpp>
#include <godot_cpp/classes/static_body2d.hpp>
//#include <godot_cpp\variant\packed_int32_array.hpp>
//#include <godot_cpp\variant\packed_vector2_array.hpp>

namespace godot {
	template <class T>
	class Ref;

	class AnimatedSprite2D;
	class CollisionPolygon2D;
	struct Vector2;
	struct MeshInstance2D;
	struct Vector3;
}

class SurfaceMesh2D : public godot::StaticBody2D
{
	GDCLASS(SurfaceMesh2D, godot::StaticBody2D)
private:
	using Super = godot::StaticBody2D;
protected:
	static constexpr int segmentCount{ 18 };
	static_assert(segmentCount >= 1, "segmentCount must be >= 0");

	static constexpr float subSegmentWidth{ 0.75f };
	static constexpr float minHeight{ 2.5f };
	//static constexpr int segmentsCount{ 17 };
	static constexpr int subSegmentCount{ 64 };
	static_assert(subSegmentCount && !(subSegmentCount& (subSegmentCount - 1)), 
		"subSegmentCount must be power of 2");
	static constexpr int curvePointSubCount{ subSegmentCount + 1 };
	static constexpr int lastSubVertex{ curvePointSubCount + 1 };
	static constexpr int midSubSegment{ lastSubVertex >> 1 };

	static constexpr float segmentWidth{ subSegmentWidth * subSegmentCount };
	static constexpr float totalWidth{ segmentWidth * segmentCount };
	static float CurveFunction(float x);

	/*static inline godot::PackedInt32Array indexesTemplate;
	static inline godot::PackedVector2Array vertsTemplate;
	static inline godot::PackedVector2Array uvsTemplate;*/

	static godot::Vector2 CalcArraysDown(godot::PackedVector2Array& verts,
		godot::PackedVector2Array& uvs, float startX = 0.f);

	static godot::Vector2 CalcArraysUp(godot::PackedVector2Array& verts,
		godot::PackedVector2Array& uvs, float startX = 0.f);

	static godot::PackedInt32Array CalcIndexesTemplate();
	static godot::PackedVector2Array CalcVertsTemplate();
	static godot::PackedVector2Array CalcUVsTemplate();

	static const godot::PackedInt32Array& GetIndexesTemplateRef();
	static const godot::PackedVector2Array& GetVertsTemplateRef();
	static const godot::PackedVector2Array& GetUVsTemplateRef();

	using segmentPairType = std::pair<godot::CollisionPolygon2D*, godot::MeshInstance2D*>;

	using segmentsArrayType = std::array<segmentPairType, segmentCount>;

	segmentsArrayType segments;
	//std::pair<godot::MeshInstance2D, godot::CollisionPolygon2D>
	void RetreiveChildren(godot::Node* owner)&;
	void ConfigureSegment(godot::Vector2& startPos, segmentPairType& pair, float startX)&;
	void ConfigureSegments(float startX)&;
public:
	//virtual void _enter_tree() override;
	static void CalcTemplateArrays();
	virtual void _ready() override;
	//virtual void _physics_process(double deltatime) override;
	static void _bind_methods();
};

