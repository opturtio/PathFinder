@echo off
SETLOCAL EnableDelayedExpansion

REM Define the HTML report directory
SET "HTML_REPORT_DIR=.\CoverageReport"
SET "COVERAGE_FILES_DIR=%HTML_REPORT_DIR%\CoverageFiles"

REM Step 1: Clean the CoverageFiles/ folder
IF EXIST "%COVERAGE_FILES_DIR%" (
    echo Cleaning existing coverage files...
    rmdir /s /q "%COVERAGE_FILES_DIR%"
)
mkdir "%COVERAGE_FILES_DIR%"

REM Step 2: Run tests and create coverage reports
echo Running dotnet test with coverage...
dotnet test --collect:"XPlat Code Coverage" --results-directory:"%COVERAGE_FILES_DIR%"

REM Step 3: Check if coverage files were created
echo Checking for coverage files in %COVERAGE_FILES_DIR%
dir %COVERAGE_FILES_DIR%

REM Step 4: Create a new HTML coverage report using ReportGenerator
IF NOT EXIST "%HTML_REPORT_DIR%" mkdir "%HTML_REPORT_DIR%"
echo Generating HTML report...
reportgenerator -reports:"%COVERAGE_FILES_DIR%\**\coverage.cobertura.xml" -targetdir:"%HTML_REPORT_DIR%" -reporttypes:Html

echo Coverage report generated at %HTML_REPORT_DIR%
ENDLOCAL
