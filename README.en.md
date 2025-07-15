<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Speech Recognition NeMo

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#launch-and-usage">Launch and Usage</a></li>
    <li><a href="#supported-models-and-languages">Supported Models and Languages</a></li>
    <li><a href="#milestones">Milestones</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>

<!-- Introduction -->
## Introduction

Speech Recognition NeMo is a package that integrates the Automatic Speech Recognition (ASR) capabilities of the NeMo Framework with ROS2 Action communication. It provides fast and accurate speech recognition.

The NVIDIA NeMo Framework is a scalable, cloud-native generative AI framework built for researchers and PyTorch developers working in Large Language Models (LLM), Multimodal Models (MM), Automatic Speech Recognition (ASR), Text-to-Speech (TTS), and Computer Vision (CV).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Getting Started -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Prerequisites
First, ensure you have the following environment set up before proceeding to the installation steps.
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation
1. Navigate to your ROS2 `src` folder.
    ```sh
    cd ~/colcon_ws/src/
    ```

2. Clone this repository．
    ```sh
    git clone -b humble-devel https://github.com/TeamSOBITS/speech_recognition_nemo.git
    ```
3. Move into the repository directory.
    ```sh
    cd speech_recognition_nemo/
    ```
4. Install dependencies. Note that this may take some time.
    ```sh
    bash install.sh
5. Compile the package.
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- Launch and Usage -->
## Launch and Usage
1. In Ubuntu settings, set the input device for sound to the microphone you intend to use.

2. Start the Action Server. Please wait for **NeMo Server is READY and waiting for requests** to appear before sending any goals.

   ```sh
   ros2 launch speech_recognition_nemo nemo_server.launch.py 
   ```
3. Start the Action Client and send the text you want to speak.

    It is recommended to set the feedback interval to at least 1.5 seconds.

    Recorded audio is saved in the **sound_file** directory.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Supported Models and Languages
speech_recognition_nemo supports the following languages:

| Supported Language  | Model Name |
| ----- | ----- |
| English | [nvidia/parakeet-tdt-0.6b-v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) (Default)|
| Japanese | [nvidia/parakeet-tdt_ctc-0.6b-ja](https://huggingface.co/nvidia/parakeet-tdt_ctc-0.6b-ja) |
| Others | Refer to [Parakeet](https://huggingface.co/collections/nvidia/parakeet-659711f49d1469e51546e021) or [Canary](https://huggingface.co/collections/nvidia/canary-65c3b83ff19b126a3ca62926) websites |

To change the language to something other than English, or to use other models, perform the following:

1. In [model_download.py](speech_recognition_nemo/model_download.py), replace **model_name** with the desired model name and run the following to download the model.
```sh
ros2 run speech_recognition_nemo model_download
```

2. Similarly, replace the **model_name** in [nemo_server.launch.py](launch/nemo_server.launch.py ) with the desired model name.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Milestones

Please refer to the [Issue page](issues-url) for current bugs and new feature requests.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## References
* [NeMo overview](https://docs.nvidia.com/nemo-framework/user-guide/latest/overview.html)
* [NeMo github](https://github.com/NVIDIA/NeMo)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/speech_recognition_nemo.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/speech_recognition_nemo/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/speech_recognition_nemo.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/speech_recognition_nemo/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/speech_recognition_nemo.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/speech_recognition_nemo/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/speech_recognition_nemo.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/speech_recognition_nemo/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/speech_recognition_nemo.svg?style=for-the-badge
[license-url]: LICENSE
