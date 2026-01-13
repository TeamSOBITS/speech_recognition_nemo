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
    <li><a href="#parameters">Parameters</a></li>
    <li><a href="#milestones">Milestones</a></li>
    <li><a href="#references">References</a></li>
  </ol>
</details>

<!-- Introduction -->
## Introduction

Speech Recognition NeMo is a package that integrates the Automatic Speech Recognition (ASR) capabilities of the NeMo Framework with ROS2 Action communication. It provides fast and accurate speech recognition.

Use with a PC equipped with a GPU is recommended.

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
    ```
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
3. Start the Action Client.
    - timeout_sec: Duration (in seconds) to keep the microphone open. If a negative value is provided, it continues to return feedback until a cancel request is sent.
    - silent_mode: When set to 'true', sound feedback is disabled at the start of detection and upon termination.
    - feedback_rate: The frequency at which intermediate speech recognition results are returned when 'use_feedback' is set to 'True' and 'vad_name' is set to 'None'. (Measured in seconds, e.g., 0.5 means every 0.5 seconds.)
    ```sh
    ros2 action send_goal /speech_recognition sobits_interfaces/action/SpeechRecognition "timeout_sec: 5 
    silent_mode: false
    feedback_rate: 0.5" -f
    ```

    Recorded audio is saved in the **sound_file** directory.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Parameters

You can specify the following parameters in **[nemo_server.launch.py](launch/nemo_server.launch.py)**.

| Parameter | Description | Default Value |
| --- | --- | --- |
| model_name | The name of the speech recognition model *| nvidia/parakeet-tdt-0.6b-v2 |
| device	 | Computing device to use (`cpu` or `cuda`). If left empty, it automatically selects GPU if available, otherwise falls back to CPU.	| "" |
| mic_volume | Sets the microphone input volume as a percentage. When finish program, the original volume will be restored. e.g., "150"	| "" |
| use_feedback | Whether to use Feedback | True |

*The following languages are supported:

  - English: [nvidia/parakeet-tdt-0.6b-v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) (Default)
  - Japanese: [nvidia/parakeet-tdt_ctc-0.6b-ja](https://huggingface.co/nvidia/parakeet-tdt_ctc-0.6b-ja)
  - Other languages: Refer to the [Parakeet](https://huggingface.co/collections/nvidia/parakeet-659711f49d1469e51546e021) and [Canary](https://huggingface.co/collections/nvidia/canary-65c3b83ff19b126a3ca62926) sites.

To change the language from English or use a different model, follow these steps:

1.  In **[model_download.py](speech_recognition_nemo/model_download.py)**, replace the **model_name** with the model you want to use and run the following command to download the model:

    ```sh
    ros2 run speech_recognition_nemo model_download
    ```

2.  Similarly, replace the **model_name** in **[nemo_server.launch.py](launch/nemo_server.launch.py)** with the name of the model you want to use.

---
The following are parameters related to Feedback.
They are only effective when `use_feedback` is set to `True`.
Changing these values will not affect the final recognition result.

| Parameter | Description | Default Value |
| --- | --- | --- |
| vad_name | The Voice Activity Detection (VAD) method used for feedback. Using VAD improves the recognition accuracy of feedback. If you select None, VAD will not be used and speech recognition will be performed at the Feedback Rate specified by the Action Client. | ten_vad |
| hop_size | The size of the chunk (fragment) of audio data processed by the VAD model. You can select 160 or 256. A smaller value increases responsiveness but also increases CPU load. | 256 |
| threshold | The probability threshold for the VAD model to detect speech. A higher value reduces false positives, but quiet or faint voices may be ignored. | 0.5 |
| min_wipe_duration | The minimum required duration of a voice to be processed for speech recognition, ignoring noise. If a section recognized as speech by VAD is shorter than this duration, it will be ignored as noise and not processed for speech recognition. | 0.2 |
| extra_audio_duration_sec | Additional audio time to include before and after the audio for each feedback. | 0.2 |
| max_speech_duration | Maximum duration (in seconds) to segment a single utterance. | 30.0 |

---
The following are parameters related to echo cancellation.

| Parameter | Description |	Default Value|
| --- | --- | --- |
| use_echo_cancel | It helps prevent the microphone from picking up audio from the speakers. | False |
| noise_suppression | Toggles the noise suppression feature. | False |
| analog_gain_control | Automatically adjusts the microphone input volume at the hardware level. It suppresses loud sounds and amplifies quiet ones to prevent clipping and improve clarity. | False |
| digital_gain_control | Automatically adjusts the input volume at the software level. It modifies the amplitude after the audio data has been digitized. | False |

  - Parameters other than `model_name`, `use_feedback`, `vad_name`, and those related to echo cancellation can be changed after the launch file is started.
      - Example: To change `min_wipe_duration` to 0.1
        ```sh
        ros2 param set /nemo_asr_action_server min_wipe_duration 0.1
        ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



## Milestones

Check the Issues page to view current bugs and feature requests.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## References
* [NeMo overview](https://docs.nvidia.com/nemo-framework/user-guide/latest/overview.html)
* [NeMo github](https://github.com/NVIDIA/NeMo)
* [TEN VAD](https://github.com/TEN-framework/ten-vad)
* [module-echo-cancel](https://www.freedesktop.org/wiki/Software/PulseAudio/Documentation/User/Modules/?utm_source=chatgpt.com#module-echo-cancel)

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
