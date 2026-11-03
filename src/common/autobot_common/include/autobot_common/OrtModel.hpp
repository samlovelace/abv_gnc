#ifndef ORT_MODEL_HPP
#define ORT_MODEL_HPP

#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <string>
#include <vector>
#include "autobot_common/Tensor.hpp"

// Thin, generic wrapper around an ONNX Runtime session: named float tensors
// in, named float tensors out. Works for an image detector or a small
// control-policy MLP without changes - task-specific pre/post-processing
// belongs in whatever consumes this, not here.
class OrtModel {
public:
    // No default member initializers here on purpose: a nested class's
    // default member initializers can't be used by a default *argument* of
    // the enclosing class's own member functions (both are deferred to
    // complete-class context, and that's circular for a type nested inside
    // the very class whose method is using it as a default argument) - GCC
    // rejects it. Aggregate `{}` still zero-inits both fields the same way.
    struct Options { bool useCuda; int numThreads; };

    explicit OrtModel(const std::string& path, const Options& opts = {})
        : mSession(env(), path.c_str(), makeOptions(opts))
    {
        Ort::AllocatorWithDefaultOptions alloc;

        for (size_t i = 0; i < mSession.GetInputCount(); ++i) {
            mInStore.emplace_back(mSession.GetInputNameAllocated(i, alloc).get());
            mInShapes.push_back(
                mSession.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        }
        for (size_t i = 0; i < mSession.GetOutputCount(); ++i)
            mOutStore.emplace_back(mSession.GetOutputNameAllocated(i, alloc).get());

        // Build the const char* arrays only after the string vectors are final.
        for (auto& s : mInStore)  mInNames.push_back(s.c_str());
        for (auto& s : mOutStore) mOutNames.push_back(s.c_str());
    }

    std::vector<Tensor> run(const std::vector<Tensor>& inputs)
    {
        if (inputs.size() != mInNames.size())
            throw std::invalid_argument("model expects " + std::to_string(mInNames.size())
                                        + " inputs, got " + std::to_string(inputs.size()));

        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        std::vector<Ort::Value> ins;
        for (const auto& t : inputs) {
            if (t.data.size() != t.numel())
                throw std::invalid_argument("tensor data size does not match its shape");
            ins.push_back(Ort::Value::CreateTensor<float>(
                mem, const_cast<float*>(t.data.data()), t.data.size(),
                t.shape.data(), t.shape.size()));
        }

        auto outs = mSession.Run(Ort::RunOptions{nullptr},
                                 mInNames.data(),  ins.data(),  ins.size(),
                                 mOutNames.data(), mOutNames.size());

        std::vector<Tensor> result;
        result.reserve(outs.size());
        for (auto& o : outs) {
            auto info = o.GetTensorTypeAndShapeInfo();
            const float* p = o.GetTensorData<float>();
            result.push_back({info.GetShape(),
                              std::vector<float>(p, p + info.GetElementCount())});
        }
        return result;
    }

    // Convenience for the common single-input case.
    std::vector<Tensor> run(const Tensor& input) { return run(std::vector<Tensor>{input}); }

    size_t numInputs() const { return mInNames.size(); }
    const std::vector<int64_t>& inputShape(size_t i) const { return mInShapes.at(i); }
    const std::vector<std::string>& outputNames() const { return mOutStore; }

private:
    static Ort::Env& env() {
        static Ort::Env e(ORT_LOGGING_LEVEL_WARNING, "ort");   // one per process
        return e;
    }
    static Ort::SessionOptions makeOptions(const Options& o) {
        Ort::SessionOptions so;
        if (o.numThreads > 0) so.SetIntraOpNumThreads(o.numThreads);
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        if (o.useCuda) { OrtCUDAProviderOptions cuda{}; so.AppendExecutionProvider_CUDA(cuda); }
        return so;
    }

    Ort::Session mSession;                       // declared first: initialized first
    std::vector<std::string> mInStore, mOutStore;   // own the name text
    std::vector<const char*> mInNames, mOutNames;   // point into the stores; what Run() needs
    std::vector<std::vector<int64_t>> mInShapes;
};

#endif // ORT_MODEL_HPP
