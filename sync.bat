@echo off
REM Get the current date and time
for /f "tokens=2 delims==" %%I in ('"wmic os get localdatetime /value"') do set datetime=%%I
set datetime=%datetime:~0,4%-%datetime:~4,2%-%datetime:~6,2%_%datetime:~8,2%-%datetime:~10,2%

REM Add all changes
git add .

REM Commit changes with current date and time
git commit -m "Auto commit: %datetime%"

REM Pull the latest changes
git pull

REM Push changes to the repository
git push

echo Commit and push completed with message: "Auto commit: %datetime%"
pause

