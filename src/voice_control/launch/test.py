import torch
from TTS.api import TTS
import soundfile as sf

def test_tts_th():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # ถ้ามีโมเดลไทย เช่น KhanomTan
    model_name = "wannaphong/khanomtan-tts-v1.0"  # ตัวอย่างโมเดลไทย
    # หรือ ถ้า Coqui มีโมเดล multilingual ไทย เช่น:
    # model_name = "tts_models/multilingual/..."  

    tts = TTS(model_name, progress_bar=False, gpu=(device == "cuda")).to(device)

    text = "สวัสดีครับ ฉันคือระบบช่วยเหลือด้วยเสียง"
    wav = tts.tts(text)  # คืนค่า numpy array
    # ถ้าต้องการบันทึกไฟล์
    sf.write("output_th.wav", wav, tts.synthesizer.output_sample_rate)
    print("Generated speech to output_th.wav")

if __name__ == "__main__":
    test_tts_th()
