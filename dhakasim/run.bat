@echo off
set JAVA_HOME=C:\Program Files\Eclipse Adoptium\jdk-21.0.11.10-hotspot
set PROJ_ROOT=%~dp0

"%JAVA_HOME%\bin\java" -cp "%PROJ_ROOT%out\artifacts\DhakaSim_jar\DhakaSim.jar;%PROJ_ROOT%libraries\commons-math3-3.6.1.jar;%PROJ_ROOT%libraries\jmathio.jar" thesisfinal.DhakaSim
