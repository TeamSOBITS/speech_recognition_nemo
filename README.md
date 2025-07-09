<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Speech Recognition NeMo

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行操作方法">実行・操作方法</a></li>
    <li><a href="#対応モデルと言語">対応モデルと言語</a></li>
    <li><a href="#録音された音声について">録音された音声について</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>

<!-- レポジトリの概要 -->
## 概要

speech_recognition_nemoは，NeMo Frameworkの自動音声認識（ASR）機能をROS2のアクション通信に対応させたものです．高速で高精度な音声認識を提供します．


NVIDIA NeMo Frameworkは，大規模言語モデル（LLM），マルチモーダルモデル（MM），自動音声認識（ASR），テキスト読み上げ（TTS），そしてコンピュータービジョン（CV）の分野に取り組む研究者やPyTorch開発者向けに構築された，スケーラブルでクラウドネイティブな生成AIフレームワークです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- セットアップ -->
## セットアップ
ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 環境条件
まず，以下の環境を整えてから，次のインストール方法に進んでください．
| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法
1. ROS2の`src`フォルダに移動します．
    ```sh
    cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    git clone -b humble-devel https://github.com/TeamSOBITS/speech_recognition_nemo.git
    ```
3. レポジトリの中へ移動します．
    ```sh
    cd speech_recognition_nemo/
    ```
4. 依存パッケージをインストールします．時間がかかるので注意．
    ```sh
    bash install.sh
5. パッケージをコンパイルします．
    ```sh
    cd ~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```sh
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- 実行・操作方法 -->
## 実行・操作方法
1. アクションサーバーを起動します．**NeMo Server is READY and waiting for requests**と表示されるまでgoalを送らずに待機してください．

   ```sh
   ros2 launch speech_recognition_nemo nemo_server.launch.py 
   ```
2. アクションクライアントを起動し，発話させたい文字を送信します．

    録音された音声は**sound_file**ディレクトリに保存されます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 対応モデルと言語
speech_recognition_nemoは以下の言語に対応しています．

| 対応言語  | モデル名 |
| ----- | ----- |
| 英語 | [nvidia/parakeet-tdt-0.6b-v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) (デフォルト)|
| 日本語 | [nvidia/parakeet-tdt_ctc-0.6b-ja](https://huggingface.co/nvidia/parakeet-tdt_ctc-0.6b-ja) |
| その他 | [Parakeet](https://huggingface.co/collections/nvidia/parakeet-659711f49d1469e51546e021)や[Canary](https://huggingface.co/collections/nvidia/canary-65c3b83ff19b126a3ca62926)のサイトを参照 |

使用言語を英語以外に変更する場合や，他のモデルを使用する場合は以下を実行してください．

1. [model_download.py](speech_recognition_nemo/model_download.py)の**model_name**を使用するモデル名に書き換えて以下を実行し，モデルをダンロードする．
```sh
ros2 run speech_recognition_nemo model_download
```

2. [nemo_server.launch.py](launch/nemo_server.launch.py )の**model_name**も同様に，使用するモデル名に書き換えてください．


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン
- [ ] Feedbackの実装

現時点のバッグや新規機能の依頼を確認するためにIssueページ をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
* [NeMo overview](https://docs.nvidia.com/nemo-framework/user-guide/latest/overview.html)
* [NeMo github](https://github.com/NVIDIA/NeMo)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


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
