import subprocess
import re
import os
import numpy as np
import wave
import glob
from scipy.signal import resample_poly

PULSEAUDIO_SOURCE_NAME_PATTERN = re.compile(r'^\s*(?:Name|名前):\s*(.+)\s*$')
PULSEAUDIO_SAMPLE_SPEC_PATTERN = re.compile(r'^\s*(?:Sample Specification|サンプル仕様):\s*(\S+)\s+(\d+)ch\s+(\d+)Hz')

def get_pulseaudio_source_info(logger):
    try:
        info = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True)
        default_source = None
        for line in info.stdout.splitlines():
            if "Default Source:" in line or "デフォルトソース:" in line:
                default_source = line.split(':', 1)[1].strip()
                break
        if not default_source:
            logger.warn("Default PulseAudio source not found.")
            return None, None, None

        list_sources = subprocess.run(['pactl', 'list', 'sources'], capture_output=True, text=True, check=True)
        blocks = []
        current_block = []
        for line in list_sources.stdout.splitlines():
            if line.strip().startswith("Source #"):
                if current_block:
                    blocks.append(current_block)
                current_block = [line]
            else:
                current_block.append(line)
        if current_block:
            blocks.append(current_block)

        for block in blocks:
            for line in block:
                m = PULSEAUDIO_SOURCE_NAME_PATTERN.match(line)
                if m and m.group(1).strip() == default_source:
                    rate, channels = parse_sample_rate_and_channels(block)
                    return default_source, rate, channels

        logger.warn(f"Could not find detailed info for source '{default_source}'.")
        return default_source, None, None

    except Exception as e:
        logger.error(f"PulseAudio source info error: {e}")
        return None, None, None

def get_default_sink(logger):
    try:
        info = subprocess.run(['pactl', 'info'], capture_output=True, text=True, check=True)
        for line in info.stdout.splitlines():
            if "Default Sink:" in line or "デフォルトシンク:" in line:
                return line.split(':', 1)[1].strip()
        logger.warn("Default PulseAudio sink not found.")
        return None
    except Exception as e:
        logger.error(f"Failed to get default sink: {e}")
        return None

def parse_sample_rate_and_channels(lines):
    for line in lines:
        m = PULSEAUDIO_SAMPLE_SPEC_PATTERN.match(line)
        if m:
            return int(m.group(3)), int(m.group(2))
    return None, None

def get_source_volume(source_name, logger):
    if not source_name:
        return None
    try:
        list_sources = subprocess.run(['pactl', 'list', 'sources'], capture_output=True, text=True, check=True)
        source_found = False
        volume_percent = None
        for line in list_sources.stdout.splitlines():
            if f"Name: {source_name}" in line or f"名前: {source_name}" in line:
                source_found = True
            if source_found:
                if line.strip().startswith("Source #"):
                    break  
                vol_match = re.findall(r'(\d+)%', line)
                if vol_match:
                    avg_volume = int(np.mean([int(v) for v in vol_match]))
                    volume_percent = f"{avg_volume}%"
                    return volume_percent

        if not source_found:
            logger.warn(f"Source '{source_name}' not found in pactl output.")
        return volume_percent

    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to get source volume for '{source_name}': {e}")
        return None

def play_sound(filename, sound_files_path, logger):
    path = os.path.join(sound_files_path, filename)
    try:
        subprocess.run(['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', path], check=True)
    except FileNotFoundError:
        logger.warn(f"ffplay not found: {filename}")
    except subprocess.CalledProcessError as e:
        logger.warn(f"Failed to play sound: {e}")
    except Exception as e:
        logger.warn(f"An error occurred during sound playback: {e}")

def resample_audio(audio_np: np.ndarray, orig_sr: int, target_sr: int, channels: int):
    if channels > 1:
        audio_np = audio_np.reshape(-1, channels)
        audio_np = audio_np.mean(axis=1)
    return resample_poly(audio_np, target_sr, orig_sr)

def save_buffer_to_wav(frames, file_path, sample_rate, channels, logger):
    if isinstance(frames, np.ndarray):
        if frames.size == 0:
            logger.warn("No frames to save.")
            return False
    elif not frames:
        logger.warn("No frames to save.")
        return False
    
    try:
        if isinstance(frames, list) and len(frames) > 0 and isinstance(frames[0], np.ndarray):
            frames_bytes = b"".join([frame.tobytes() for frame in frames])
        elif isinstance(frames, list) and len(frames) == 1 and isinstance(frames[0], bytes):
                frames_bytes = frames[0]
        elif isinstance(frames, np.ndarray):
                frames_bytes = frames.tobytes()
        else:
            logger.error("Unsupported frames type.")
            return False

        with wave.open(file_path, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(frames_bytes)
        return True
    except Exception as e:
        logger.error(f"Failed to save WAV file: {e}")
        return False

def cleanup_wav_files(directory, logger):
    wip_files = glob.glob(os.path.join(directory, "*.wav"))
    for f in wip_files:
        try:
            os.remove(f)
            logger.info(f"Removed old WAV file: {f}")
        except Exception as e:
            logger.warn(f"Failed to remove WAV file {f}: {e}")

def cleanup_pulse_audio(logger, source_to_modify, mic_volume, original_mic_volume, original_default_sink, aec_module_index):
    if source_to_modify and original_mic_volume and mic_volume:
        try:
            subprocess.run(['pactl', 'set-source-volume', source_to_modify, original_mic_volume], check=True, capture_output=True)
            logger.info(f"Restored volume for '{source_to_modify}' to its original value: {original_mic_volume}.")
        except subprocess.CalledProcessError as e:
            logger.warn(f"Failed to restore volume for '{source_to_modify}': {e.stderr.decode().strip()}")

    if original_default_sink and aec_module_index is not None:
        try:
            subprocess.run(['pactl', 'set-default-sink', original_default_sink], check=True, capture_output=True)
            logger.info(f"Restored default sink to '{original_default_sink}'.")
        except subprocess.CalledProcessError as e:
            logger.warn(f"Failed to restore default sink: {e.stderr.decode().strip()}")

    if aec_module_index is not None:
        try:
            subprocess.run(['pactl', 'unload-module', str(aec_module_index)], check=True, capture_output=True)
            logger.info(f"Unloaded AEC module index {aec_module_index}.")
        except subprocess.CalledProcessError as e:
            logger.warn(f"Failed to unload AEC module: {e.stderr.decode().strip()}")

def configure_pulseaudio(logger, use_echo_cancel, noise_suppression, analog_gain_control, digital_gain_control, mic_volume):
    config = {
        "source_name": "default",
        "sample_rate": 44100,
        "channels": 2,
        "aec_module_index": None,
        "original_default_sink": None,
        "source_to_modify": None,
        "original_mic_volume": None,
        "echo_cancel_source": None,
        "echo_cancel_sink": None,
        "use_echo_cancel": use_echo_cancel
    }

    source_name, sample_rate, channels = get_pulseaudio_source_info(logger)
    if source_name is None:
        logger.warn("Failed to get default microphone. Using fallback settings.")
    else:
        config["source_name"] = source_name
        config["sample_rate"] = sample_rate if sample_rate is not None else 44100
        config["channels"] = channels if channels is not None else 2
    
    logger.info(f"Microphone: {config['source_name']}, Sample rate: {config['sample_rate']} Hz, Channels: {config['channels']}")
    
    if config["use_echo_cancel"]:
        logger.info("Echo cancellation is enabled.")
        config["echo_cancel_source"] = "mic_aec"
        config["echo_cancel_sink"] = "speaker_aec"
        config["original_default_sink"] = get_default_sink(logger)

        try:
            aec_args = (
                f"noise_suppression={int(noise_suppression)} "
                f"analog_gain_control={int(analog_gain_control)} "
                f"digital_gain_control={int(digital_gain_control)}"
            )
            result = subprocess.run(
                [
                    'pactl', 'load-module', 'module-echo-cancel',
                    f"source_name={config['echo_cancel_source']}",
                    f"sink_name={config['echo_cancel_sink']}",
                    'aec_method=webrtc',
                    f'aec_args="{aec_args}"'
                ],
                check=True, capture_output=True, text=True
            )
            config["aec_module_index"] = int(result.stdout.strip())
            logger.info(f"AEC module loaded with index: {config['aec_module_index']}")
            
            subprocess.run(['pactl', 'set-default-sink', config["echo_cancel_sink"]], check=True)
            logger.info(f"Default sink set to '{config['echo_cancel_sink']}'")
            config["source_to_modify"] = config["echo_cancel_source"]
            config["original_mic_volume"] = get_source_volume(config["source_to_modify"], logger)
            if mic_volume:
                subprocess.run(['pactl', 'set-source-volume', config["source_to_modify"], mic_volume], check=True)
                logger.info(f"Microphone volume set to {mic_volume} for '{config['source_to_modify']}'")
        
        except (subprocess.CalledProcessError, ValueError) as e:
            logger.error(f"Failed to load or configure AEC module: {e}")
            config["use_echo_cancel"] = False
            config["source_to_modify"] = config["source_name"]
            config["original_mic_volume"] = get_source_volume(config["source_to_modify"], logger)
            if mic_volume:
                subprocess.run(['pactl', 'set-source-volume', config["source_to_modify"], mic_volume], check=True)
                logger.info(f"Fell back to default source. Microphone volume set to {mic_volume} for '{config['source_to_modify']}'")
    else:
        config["source_to_modify"] = config["source_name"]
        config["original_mic_volume"] = get_source_volume(config["source_to_modify"], logger)
        if mic_volume:
            subprocess.run(['pactl', 'set-source-volume', config["source_to_modify"], mic_volume], check=True)
            logger.info(f"Microphone volume set to {mic_volume} for '{config['source_to_modify']}'")
            
    logger.info(f"Original mic volume for '{config['source_to_modify']}' was {config['original_mic_volume']}")
    
    return config