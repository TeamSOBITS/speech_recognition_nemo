import nemo.collections.asr as nemo_asr

def main():
    print("Loading model...")
    # English model
    asr_model = nemo_asr.models.ASRModel.from_pretrained(model_name="nvidia/parakeet-tdt-0.6b-v2")

    # Japanese model
    # nvidia/parakeet-tdt_ctc-0.6b-ja
    print("Model load finished\n")

if __name__ == '__main__':
    main()