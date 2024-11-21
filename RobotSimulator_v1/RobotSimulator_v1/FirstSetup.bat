@ECHO OFF
echo Starting first setup of enviroment...
py -m venv env
echo Enviroment created..
%cd%\env\Scripts\pip.exe install -r requirements.txt
echo Packages installed...
echo Done!
PAUSE

