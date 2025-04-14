#pragma once

#include <vector>


namespace linalg
{

template<typename T>
std::vector<T> linspace(T start, T stop, int num);

template<typename T>
T norm(const std::vector<T> &v);

template<typename T>
std::vector<T> cross(const std::vector<T> &v1, const std::vector<T> &v2);

}; // namespace linalg

#include "arena_core/math/impl/utils/linalg.inl"
