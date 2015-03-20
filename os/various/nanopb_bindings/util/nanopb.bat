@echo off

set mydir=%cd%\%1
set scriptdir=%~dp0

cd %scriptdir%\..\..\nanopb\generator-bin\
protoc --proto_path=%mydir% --nanopb_out=%mydir% %mydir%\%2
