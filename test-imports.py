import asyncio
import re
import threading
import keyboard
import subprocess
import time
import platform
import speech_recognition as sr
from langchain_ollama import OllamaLLM
from langchain_core.prompts import ChatPromptTemplate
import RPi.GPIO as GPIO
import logging