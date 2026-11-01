#ifndef TENSOR_HPP
#define TENSOR_HPP

#include <cstdint>
#include <vector>

struct Tensor {
    std::vector<int64_t> shape;   // e.g. {1, 3, 640, 640}
    std::vector<float>   data;

    size_t numel() const {
        size_t n = 1;
        for (auto d : shape) n *= static_cast<size_t>(d);
        return n;
    }
};

#endif // TENSOR_HPP
