> **IMPORTANT**
Please make sure that you have gone through the [Convert a PyTorch model to an ONNX model](https://github.com/UMich-CURLY/lab_wiki/wiki/Convert-a-Pytorch-Model-to-an-ONNX-model) tutorial and have a valid ONNX model ready to use. After this tutorial, you may use [TensorRT_PROJECT_USE](https://github.com/Tingjun-Li/TensorRT_PROJECT_USE) as a real case exercise. 

# Official Tutorial
The tutorials here assume that you already have a workable ONNX model on your computer. The offical guides provided by NVIDIA are long, but they are worthy to read (or at least have a basic idea about procedure of implementing TensorRT), especially the installation part. **Debian Installatioin** is recommended for first time user because it comes with useful sample codes. Also, please verify if your cuda version and cudnn version is compatible with the TensorRT version you are going to install.

In Jetson AGX Xavier DEVELOPER KIT (The latest one released before 2021.7.28), it has TensorRT 7.1.3 installed:
- **dependencies:** CUDA 10.2 or 11.0 RC, cuDNN 8.0.0 Preview
- [TensorRT 7.1.3 Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-713/install-guide/index.html)
- [TensorRT 7.1.3 Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-713/developer-guide/index.html#c_topics)

You may also install the following version (version older than this is not recommended):
- **dependencies:** CUDA 9.0, 10.0, or 10.2, cuDNN 7.6.5
- [TensorRT 7.0.0 Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-700/tensorrt-install-guide/index.html)
- [TensorRT 7.0.0 Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/archives/tensorrt-700/tensorrt-developer-guide/index.html#c_topics)

# Sample Case Study
## 1. Introduction
TensorRT is a powerful tool in network acceleration. It can boost the frequency of a network from 300Hz to 900Hz. However, using it for the first time will not be easy. Let's get started!

In this case study, one can open the file `sampleOnnxMNIST` in folder `/usr/src/tensorrt/samples/sampleOnnxMNIST/` (this file can also be found in offical github repo: [sampleOnnxMNIST](https://github.com/NVIDIA/TensorRT/blob/release/7.0/samples/opensource/sampleOnnxMNIST/sampleOnnxMNIST.cpp)). **The content in it may vary between different versions.** One should also change their code accordingly if they are using different versions of TensorRT on different machines. Luckily, the changes are mostly **minor** between similar version numbers.

The structure of a TensorRT acceleration procedure can be interpreted as:
- **First time**:
1. Build an engine from a pre-trained model
2. Infer and verify a result from the engine and an input
3. Serialize the engine and save it on the disk for later use

>**IMPORTANT**: Different machines have different structures in saving the engine. Thus, a TensorRT engine generated on machine A may not work on machine B. One should serialize the engine and use it only on the computer where the model is serialized. (**Note given by NVIDIA:** Serialized engines are not portable across platforms or TensorRT versions. Engines are specific to the exact GPU model they were built on (in addition to the platforms and the TensorRT version).)

- **Later**:
1. Deserialize a pre-saved engine and build it
2. Infer and verify a result from the engine and an input
3. Repeat step 2 to run the engine for different inputs

## 2. Before running the official example
To run this example, you need to first compile it by running the `Makefile` in `/usr/src/tensorrt/samples/` or in `/usr/src/tensorrt/samples/sampleOnnxMNIST/`. The executable file `sampleOnnxMNIST` can then be found in `/usr/src/tensorrt/bin`. After this, one needs to download MNIST data by switching to `/usr/src/tensorrt/samples/data/mnist`, and then run:
```
$ sudo python3 download_pgms.py
```
It may take some time to finish the download procedure. If you saw the following warning message:
```
urllib.error.HTTPError: HTTP Error 503: Service Unavailable
```
You just need to run it again for several times or close and save the `download_pgms.py` file.

## 3. Entering into the main code
The official [README.md](https://github.com/NVIDIA/TensorRT/blob/release/7.0/samples/opensource/sampleOnnxMNIST/README.md) file is useful and introduced main steps in using TensorRT.

### 3.1 Build an engine
Every line inside `bool SampleOnnxMNIST::build()` helps in generating an engine. If this is the first time for you to generate the engine, this is a necessary step. 
>**NOTE** In some version of TensorRT (such as TensorRT 7.1.3), you may need to use `sample::gLog*` instead of purely `gLog*`, e.g. use `sample::gLogger` instead of `gLogger`. You will also receive `undefined` error if you are using a wrong variable.

First, let's create a `builder` object:
```
auto builder = SampleUniquePtr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(gLogger.getTRTLogger()));
```

Then the `network` object, which will be in storing the network:
```
const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);     
auto network = SampleUniquePtr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
```

And also the `config` object:
```
auto config = SampleUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
```

There are multiple ways to build an engine. TensorRT allows users to construct a Network from draft, but here we prefer to use a pre-trained ONNX model. One can use class `nvonnxparser` to parse an ONNX model. Let's create a `parser`:
```
 auto parser = SampleUniquePtr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, gLogger.getTRTLogger()));
```

After all of above, we can construct the network from the parse:
```
auto constructed = constructNetwork(builder, network, config, parser);
```
In function `constructNetwork`, one can see the following lines:
```
if (mParams.fp16) {...}
if (mParams.int8) {...}
```
It means that if the user runs the program and specifies that he or she would like to trade accuracy for more speed, he can choose the to construct the network using floating points 16 or even int_8 type (which is not recommended).

Then we can build an TensorRT engine from the network constructed above:
```
mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(builder->buildEngineWithConfig(*network, *config), samplesCommon::InferDeleter());
```

`mEngine` is defined as a private object in `class SampleOnnxMNIST` as
```
std::shared_ptr<nvinfer1::ICudaEngine> mEngine; //!< The TensorRT engine used to run the network
``` 

The engine will be stored in the buffer when the program is running. However, once the program stops, it will be destructed unless you save it on the disk, which is also known as **serialization**. 

### 3.2 Run inference
Lines in `bool SampleOnnxMNIST::infer()` are used to run inference. It is necessary to load the input in a way that is readable for TensorRT engine. First let's create a `buffers`:
```
samplesCommon::BufferManager buffers(mEngine, mParams.batchSize);
```
> **Note**: In some version of TensorRT (such as 7.1.3 on Jetson AGX Xavier Develop Kit), you need to use the following sentence instead of the one above:
```
samplesCommon::BufferManager buffers(mEngine);
```

Then run the following to load and store input data to the object `buffers`:
```
processInput(buffers);
```
Function `processInput()` is just a way to save the the data on to the host. The most important part of this function is:
```
float* hostDataBuffer = static_cast<float*>(buffers.getHostBuffer(mParams.inputTensorNames[0]));
for (int i = 0; i < inputH * inputW; i++)
{
    hostDataBuffer[i] = 1.0 - float(fileData[i] / 255.0);
}
```


Copy the input to CUDA device:
```
buffers.copyInputToDevice();
```

Execute the network:
```
context->executeV2(buffers.getDeviceBindings().data());
```

Copy the output back to Host (CPU):
```
buffers.copyOutputToHost();
```

Verify the output:
```
verifyOutput(buffers);
```
You can write your own function to verify the output.

### 3.3 Serialize the engine for later use:
If the output shows exactly what you expected, then you can serialize the engine. Because generating a network by parsing or from draft takes a considerable amount of time, we can deserialize an engine which is serialized in previous runs to avoiding building the engine we runs it. By adding the following function, the TensorRT engine will be saved in your desire path:
```
bool SampleOnnxToTensorRT::serialize() {
    nvinfer1::IHostMemory *serializedModel = mEngine->serialize();
    std::string serialize_str;
    std::ofstream serialize_output_stream;
    serialize_str.resize(serializedModel->size());
    memcpy((void*)serialize_str.data(), serializedModel->data(), serializedModel->size());
    serialize_output_stream.open("<save_path>/<engine_name>.trt");
    
    serialize_output_stream << serialize_str;
    serialize_output_stream.close();
    serializedModel->destroy();
    
    std::cout << "Successfully serialized the engine" << std::endl;
    return true;
}
```
Of course, you need to call this function like `sample.serialize()` to serialize it.

### 3.4 Deserialize the engine and build it:
Once a serialized engine is ready (**Note:** please do not copy the engine across different machines, it may not work), we can deserialize and build the engine by:
```
bool SampleOnnxToTensorRT::buildFromSerializedEngine()
{
    /// REMARK: we can deserialize a serialized engine if we have one:
    // -----------------------------------------------------------------------------------------------------------------------
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(sample::gLogger);
    std::string cached_path = "<saved_path>" + "<engine_name>.trt";
    std::ifstream fin(cached_path);
    std::string cached_engine = "";
    while (fin.peek() != EOF) {
        std::stringstream buffer;
        buffer << fin.rdbuf();
        cached_engine.append(buffer.str());
    }
    fin.close();
    mEngine = std::shared_ptr<nvinfer1::ICudaEngine> (
                                    runtime->deserializeCudaEngine(cached_engine.data(), cached_engine.size(), nullptr),
                                    samplesCommon::InferDeleter());
    if (!mEngine)
    {
        return false;
    }
    
    context = SampleUniquePtr<nvinfer1::IExecutionContext>(mEngine->createExecutionContext());
    if (!context)
    {
        return false;
    }
    
    std::cout << "Successfully built the engine and made the context" << std::endl;

    return true;
}
```
Then you can call the function by using `sample.buildFromSerializedEngine()`.

## 4. Real Case Exercise
One can try to use the code in repository [TensorRT_PROJECT_USE](https://github.com/Tingjun-Li/TensorRT_PROJECT_USE) to generate a TensorRT engine for the contact estimator network generated in page [Convert a PyTorch Model to an ONNX model](https://github.com/UMich-CURLY/lab_wiki/wiki/Convert-a-Pytorch-Model-to-an-ONNX-model). In this repository, you can also find how to write a CMakefile to link the libraries and header files. 

# Recourse Links:
TensorRT: https://docs.nvidia.com/deeplearning/tensorrt/archives