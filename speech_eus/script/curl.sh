curl -X POST \
--data-binary @'/home/tam/catkin_ws/src/speech_eus/audio/recording.wav' \
--header 'Content-Type: audio/l16; rate=16000;' \
'https://www.google.com/speech-api/v2/recognize?output=json&lang=eu-ES&client=chromium&maxresults=1&key=AIzaSyD2qCl5xoA0Y5jt5inElKO6glOllE9pkNc'
