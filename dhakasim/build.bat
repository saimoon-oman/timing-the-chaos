@echo off
setlocal enabledelayedexpansion

set JAVA_HOME=C:\Program Files\Eclipse Adoptium\jdk-21.0.12.8-hotspot
set PROJ_ROOT=%~dp0

echo Building DhakaSim...

REM Use subst to avoid spaces in paths
subst T: "%PROJ_ROOT%" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo Failed to create T: drive mapping. Try running as Administrator.
    exit /b 1
)

set SRC_DIR=T:\src
set OUT_DIR=T:\out\production\DhakaSim
set LIB_DIR=T:\libraries
set JAR_DIR=T:\out\artifacts\DhakaSim_jar

REM Clean output
if exist "%OUT_DIR%" rmdir /s /q "%OUT_DIR%"
mkdir "%OUT_DIR%" 2>nul

REM Generate filelist (paths have no spaces on T: drive)
set FILELIST=%TEMP%\dhakasim_files.txt
if exist "%FILELIST%" del "%FILELIST%"
for /r "%SRC_DIR%" %%f in (*.java) do echo %%f >> "%FILELIST%"

REM Compile using filelist
"%JAVA_HOME%\bin\javac" -cp "%LIB_DIR%\commons-math3-3.6.1.jar;%LIB_DIR%\jmathio.jar" -d "%OUT_DIR%" @"%FILELIST%"
set JAVAC_RESULT=%ERRORLEVEL%

REM Clean up subst
subst T: /d >nul 2>&1

if %JAVAC_RESULT% neq 0 (
    echo Compilation FAILED
    exit /b 1
)
echo Compilation SUCCESSFUL

REM Build JAR from the actual path
set REAL_JAR_DIR=%PROJ_ROOT%out\artifacts\DhakaSim_jar
if not exist "%REAL_JAR_DIR%" mkdir "%REAL_JAR_DIR%"
copy /y "%PROJ_ROOT%src\META-INF\MANIFEST.MF" "%TEMP%\MANIFEST.MF" >nul
"%JAVA_HOME%\bin\jar" cfm "%REAL_JAR_DIR%\DhakaSim.jar" "%TEMP%\MANIFEST.MF" -C "%PROJ_ROOT%out\production\DhakaSim" .
echo JAR created: %REAL_JAR_DIR%\DhakaSim.jar

endlocal
