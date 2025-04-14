#pragma once

#include <vector>


namespace linalg
{

template<typename T>
std::vector<T> linspace(T start, T stop, int num)
{
    std::vector<T> result;
    T step = static_cast<T>((stop - start) / (num - 1));
    for (int i = 0; i < num; i++) {
        result.push_back(start + i * step);
    }
    return result;
}

template<typename T>
T norm(const std::vector<T> &v)
{
    T result = 0;
    for (auto &i : v) {
        result += i * i;
    }
    return sqrt(result);
}

template<typename T>
std::vector<T> cross(const std::vector<T> &v1, const std::vector<T> &v2)
{
    std::vector<T> result;
    result.push_back(v1[1] * v2[2] - v1[2] * v2[1]);
    result.push_back(v1[2] * v2[0] - v1[0] * v2[2]);
    result.push_back(v1[0] * v2[1] - v1[1] * v2[0]);
    return result;
}
  
}; // namespace linalg
