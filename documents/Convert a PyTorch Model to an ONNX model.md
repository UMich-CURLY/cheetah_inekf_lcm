**THIS STEP IS REQUIRED BEFORE MOVING ON TO TENSORRT**

# Unofficial Tutorial
You can go through steps 1 to 3 in the following link. But please do **NOT** rush to follow the steps in it before you identified the necessity of each step. Also, you may **IGNORE** the steps about TensorRT installation or tutorial for now. 

[How to Convert a Model from PyTorch to ONNX](https://learnopencv.com/how-to-convert-a-model-from-pytorch-to-tensorrt-and-speed-up-inference/)

# Real Case Study
## Introduction
The goal of this tutorial is to convert a pre-trained PyTorch model to an ONNX model. This ONNX model can later be used in TenosorRT network acceleration [LINK]. Here we use the contact estimator network as an example. Most of the steps are inspired by the unofficial tutorial above, but in this example, we don't need to install OpenCV and we will skip the input preprocessing step. 

## Required packages 

If one is using **Jetson** devices (such as Jetson AGX Xavier), CUDA, cuDNN and TensorRT are automatically installed during Jetson setup, you may skip this step. If one used debian installation, he (she) can check the versions of those three installations by typing:
```
dpkg -l | grep TensorRT
dpkg -l | grep cudnn
dpkg -l | grep cuda
```

(lower versions of ONNX and Pytorch may work, but not recommend)
- [ONNX 1.9.0](https://pypi.org/project/onnx/)
- [Pytorch 1.8.1](https://pytorch.org/get-started/locally/)
- [CUDA 10.2](https://docs.nvidia.com/cuda/archive/10.2/index.html) (**Debian Installation is recommended**)
- [cuDNN 7.6.5](https://developer.nvidia.com/rdp/cudnn-archive) [Scroll down and find **Download cuDNN v7.6.5 (November 18th, 2019), for CUDA 10.2**] (**Debian Installation is recommended**)

The above versions are chosen according to the compatibility of [TensorRT 7.0.0](https://docs.nvidia.com/deeplearning/tensorrt/release-notes/tensorrt-7.html#rel_7-0-0). For other version of TensorRT, you should always look for compatible **cuda** versions and **cudnn** versions. NVIDIA recommends that one should use the **exact** version of **cudnn** shown on the compatibility list of **TensorRT**



In our example we also used the following packages:
- [argparse](https://pypi.org/project/argparse/)
- [PyYAML](https://pypi.org/project/PyYAML/)
- [Netron](https://pypi.org/project/netron/)

## Required data and model
- [Input data](https://drive.google.com/file/d/1I5yS_FmGt2GYWoedOUfdJLe1qHFX1X2b/view?usp=sharing)
- [Input data label](https://drive.google.com/file/d/1BRoOgABONvl3w3AWfixGSl1A17JSTmsf/view?usp=sharing)
- [Pre-trained model](https://drive.google.com/file/d/15UrYNYWFs6hQTqUC35F8lSDS_AME0dFb/view?usp=sharing)

## Code
Code can be found in:

https://github.com/UMich-CURLY/deep-contact-estimator/blob/lcm_cnn_interface_Jetson/utils/torch_to_onnx.py

### Import
```
import sys
sys.path.append("..")
import torchvision
import torch
from torch.autograd import Variable
import onnx
import argparse
from src.contact_cnn import *
from utils.data_handler import *
import os
import yaml
```

### Setup device and load path
Remember to change the path according to your saved folder. In this example, we are using `.yaml` to save the path.
```
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
parser = argparse.ArgumentParser(description='Test the contcat network')
parser.add_argument('--config_name', type=str,
                    default=os.path.dirname(os.path.abspath(__file__)) + '/../config/test_params.yaml')
args = parser.parse_args()
config = yaml.load(open(args.config_name))
```

### Load the model
Remember to check if the `model_load_path` matches the path where you saved your model, as well as the name of the model. `contact_cnn` is the model class we created, and the detail can be found [contact_cnn.py](https://github.com/UMich-CURLY/deep-contact-estimator/blob/master/src/contact_cnn.py).
```
model = contact_cnn()
checkpoint = torch.load(config['model_load_path'])
model.load_state_dict(checkpoint['model_state_dict'])
model = model.eval().to(device)
```

### Create an input batch
In our case, the size of input batch is 1. We will generate a batch using `test_onnx.npy` and `test_label_onnx.npy`. These two data contains only one input matrix of size [150 x 54]. `contact_dataset()` and `DataLoader()` are methods to create a batch. The detail for them can be found in [data_handler.py](https://github.com/UMich-CURLY/deep-contact-estimator/blob/master/utils/data_handler.py).

```
test_data = contact_dataset(data_path=config['data_folder']+"test_onnx.npy",\
                            label_path=config['data_folder']+"test_label_onnx.npy",\
                            window_size=config['window_size'], device=device)
test_dataloader = DataLoader(dataset=test_data, batch_size=1)

# Load input and generate output:
for i in test_dataloader:
    print(i['data'].shape)
    input = i['data']
    output = model(input)
```

### Convert the model to ONNX format
Use the model and input created above to generate an `.onnx` model. Remember to give name to the `input_names` and `output_names`. These names will be saved in the `.onnx` model and can later be used in NVIDIA TensorRT accelerator.

```
ONNX_FILE_PATH = '<your_path>/0616_2blocks_best_val_loss.onnx'
torch.onnx.export(model, input, ONNX_FILE_PATH, input_names=['input'], output_names=['output'], export_params=True)

onnx_model = onnx.load(ONNX_FILE_PATH)
onnx.checker.check_model(onnx_model)
```

### Verify the model structure:
You can run **Netron** (run: `netron` in your terminal and click the link), and load the model to verify if the input size, output size and model structure is what you expect. In our case, the model looks like: [contact estimator model](https://drive.google.com/file/d/1-6wPju11C3mHWKQY6w3ESL1e-uwC1Prr/view?usp=sharing)