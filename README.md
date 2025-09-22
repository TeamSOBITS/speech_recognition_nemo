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
    <li><a href="#パラメータ">パラメータ</a></li>
    <li><a href="#録音された音声について">録音された音声について</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>

<!-- レポジトリの概要 -->
## 概要

Speech Recognition NeMoは，NeMo Frameworkの自動音声認識（ASR）機能をROS2のアクション通信に対応させたものです．高速で高精度な音声認識を提供します．

GPUを搭載したPCでの使用を推奨します．

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
1. Ubuntuの設定で，サウンドの入力デバイスを使用するマイクに設定する

2. アクションサーバーを起動します．**NeMo Server is READY and waiting for requests**と表示されるまでgoalを送らずに待機してください．

   ```sh
   ros2 launch speech_recognition_nemo nemo_server.launch.py 
   ```
3. アクションクライアントを起動し，発話させたい文字を送信します．

    録音された音声は**sound_file**ディレクトリに保存されます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## パラメータ
[nemo_server.launch.py](launch/nemo_server.launch.py)では以下のパラメータを指定できます．

| パラメータ | 説明 | デフォルト値 |
| --- | --- | --- |
| model_name | 音声認識モデルの名前 *| nvidia/parakeet-tdt-0.6b-v2 |
| use_feedback | Feedbackを使用するかどうか | True |


*以下の言語に対応しています．
  - 英語: [nvidia/parakeet-tdt-0.6b-v2](https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2) (デフォルト)
  - 日本語: [nvidia/parakeet-tdt_ctc-0.6b-ja](https://huggingface.co/nvidia/parakeet-tdt_ctc-0.6b-ja)
  - その他: [Parakeet](https://huggingface.co/collections/nvidia/parakeet-659711f49d1469e51546e021)や[Canary](https://huggingface.co/collections/nvidia/canary-65c3b83ff19b126a3ca62926)のサイトを参照 

  使用言語を英語以外に変更する場合や，他のモデルを使用する場合は以下を実行してください．

1. [model_download.py](speech_recognition_nemo/model_download.py)の**model_name**を使用するモデル名に書き換えて以下を実行し，モデルをダンロードする．
    ```sh
    ros2 run speech_recognition_nemo model_download
    ```

2. [nemo_server.launch.py](launch/nemo_server.launch.py )の**model_name**も同様に，使用するモデル名に書き換えてください．

\
以下はFeedbackに関するパラメータです．
`use_feedback`が`True`のときのみ有効です．
以下の値を変更しても最終認識結果には影響しません．

| パラメータ | 説明 | デフォルト値 |
| --- | --- | --- |
| vad_name | フィードバックの際に使用する音声アクティビティ検出(VAD)の手法．VADの使用によりフィードバックの認識精度が向上する．Noneを選択するとVADを使用せずAction Clientで指定したFeedback Rateの秒数ごとに音声認識を行う． | ten_vad |
| hop_size | VADモデルが音声データを処理するチャンク（断片）のサイズ．160 or 256を選択可能．値が小さいほど応答性が上がるが，CPU負荷が増える | 256 |
| threshold | VADモデルが音声を検出するための確率のしきい値．値を高くすると誤検出が減るが，かすれた声や小さな声が無視される可能性がある | 0.5 |
| min_wipe_duration | ノイズを無視し音声認識するために必要な声の最短の長さ．VADが発話と認識した区間がこの秒数より短い場合，ノイズとして無視され音声認識の処理を行わない． | 0.2 |
| extra_audio_duration_sec | フィードバックごとに音声の前後に含める追加のオーディオ時間 | 0.2 | 

- `model_name`, `use_feedback`, `vad_name`以外のパラメータはlaunchファイル起動後でも変更可能です．
  - 例：`min_wipe_duration`を0.1に変更する場合
    ```sh
    ros2 param set /nemo_asr_action_server min_wipe_duration 0.1
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## マイルストーン

現時点のバッグや新規機能の依頼を確認するためにIssueページ をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## 参考文献
* [NeMo overview](https://docs.nvidia.com/nemo-framework/user-guide/latest/overview.html)
* [NeMo github](https://github.com/NVIDIA/NeMo)
* [TEN VAD](https://github.com/TEN-framework/ten-vad)

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
