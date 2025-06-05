# speech_recognition_nemo
parakeet-tdt-0.6b-v2 is a 600-million-parameter automatic speech recognition (ASR) model designed for high-quality English transcription, featuring support for punctuation, capitalization, and accurate timestamp prediction. Try Demo here: https://huggingface.co/spaces/nvidia/parakeet-tdt-0.6b-v2

This XL variant of the FastConformer [1] architecture integrates the TDT [2] decoder and is trained with full attention, enabling efficient transcription of audio segments up to 24 minutes in a single pass. The model achieves an RTFx of 3380 on the HF-Open-ASR leaderboard with a batch size of 128. Note: RTFx Performance may vary depending on dataset audio duration and batch size.

https://huggingface.co/nvidia/parakeet-tdt-0.6b-v2



作成中、しばし待たれよ
