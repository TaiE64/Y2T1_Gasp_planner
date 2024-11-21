@ECHO OFF 
%cd%\env\Scripts\python.exe RobotSimulator.py runserver
if %ERRORLEVEL% neq 0 echo Failed to run, have you setup the enviroment?
PAUSE