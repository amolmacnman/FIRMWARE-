@echo off
REM MISRA C:2012 + static analysis for a Smart Wagon project (GATEWAY or SUBNODE).
REM Usage:  compliance\run_cppcheck.bat GATEWAY   (run from the FIRST_TRIAL root)
setlocal
set PROJ=%1
if "%PROJ%"=="" set PROJ=GATEWAY
set BOARD=nrf54l15dk/nrf54l15/cpuapp

echo == 1) build %PROJ% (export compile_commands.json) ==
call west build -b %BOARD% --sysbuild %PROJ% -- -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo == 2) cppcheck + MISRA ==
cppcheck --project=%PROJ%\build\compile_commands.json ^
         --addon=compliance\misra.json ^
         --enable=all --inline-suppr ^
         --suppress=missingIncludeSystem ^
         --suppressions-list=compliance\misra-suppress.txt ^
         --output-file=%PROJ%-cppcheck.txt

echo == 3) clang-tidy ==
clang-tidy -p %PROJ%\build %PROJ%\src\*.c > %PROJ%-clangtidy.txt 2>&1

echo Done. Review %PROJ%-cppcheck.txt and %PROJ%-clangtidy.txt
endlocal
