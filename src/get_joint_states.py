import speech_recognition as sr
import os
from dotenv import load_dotenv
import time
import spacy

# Load environment variables from .env file
load_dotenv()

# Load spaCy model
nlp = spacy.load("en_core_web_sm")

def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`."""
    with microphone as source:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    print("Recognizing...")
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        response["error"] = "Unable to recognize speech"

    return response

def interpret_prompt(prompt):
    """Interpret the prompt using spaCy."""
    doc = nlp(prompt)
    
    # Extract the action and color from the prompt
    return 

if __name__ == "__main__":
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    speech_response = recognize_speech_from_mic(recognizer, microphone)
    if speech_response["success"]:
        print(f"Transcription: {speech_response['transcription']}")
        try:
            prompt = (
                "You are controlling a robot manipulator. The robot can perform tasks such as picking up objects of different colors. "
                "Interpret the following command and provide the action in the format 'action: color'.\n"
                f"Command: {speech_response['transcription']}\n"
                "Response format: 'pick up: color' or 'unknown command'."
            )
            interpretation = interpret_prompt(prompt)
            print(f"Interpreted command: {interpretation}")
        except Exception as e:
            print(f"Error: {e}")
    else:
        print(f"Error: {speech_response['error']}")