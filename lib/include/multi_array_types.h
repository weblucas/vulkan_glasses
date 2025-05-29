#ifndef MESH_DEPTH_EVALUATOR_MULTI_ARRAY_TYPES_H_
#define MESH_DEPTH_EVALUATOR_MULTI_ARRAY_TYPES_H_

#undef H5_USE_BOOST
#define H5_USE_BOOST

#include <boost/multi_array.hpp>

typedef boost::multi_array<unsigned char, 3> MArray3u;
typedef MArray3u::array_view<2>::type MArray3u_view2;
typedef MArray3u::index MArray3u_index;

typedef boost::multi_array<float, 2> MArray2f;
typedef boost::multi_array<double, 2> MArray2d;
typedef boost::multi_array<float, 3> MArray3f;
typedef MArray3f::array_view<2>::type MArray3f_view2;
typedef MArray3f::array_view<3>::type MArray3f_view3;
typedef boost::multi_array<int, 2> MArray2i;

//#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
//typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
//typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
//typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

//#define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)         \
//typedef Matrix<Type, Size, Dynamic> Matrix##Size##X##TypeSuffix;  \
//typedef Matrix<Type, Dynamic, Size> Matrix##X##Size##TypeSuffix;

//#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
//EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2) \
//EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3) \
//EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4) \
//EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Dynamic, X) \
//EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 2) \
//EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 3) \
//EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

// EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int,                  i)
// EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float,                f)

#endif
