@echo off
REM Copy HAL headers into this project. No PowerShell needed.
REM
REM 1. Download STM32CubeF4 from https://www.st.com/en/embedded-software/stm32cubef4.html
REM 2. Unzip it. You get a folder like STM32Cube_FW_F4_1.28.0
REM 3. Edit the line below: set SOURCE_INC= to the Inc folder inside that package.
REM    Example (change version if yours is different):
REM    set "SOURCE_INC=C:\Users\Arshman Hassan\Desktop\STM32Cube_FW_F4_1.28.0\Drivers\STM32F4xx_HAL_Driver\Inc"
REM 4. Save this file and double-click this .bat to run.

set "PROJECT_INC=%~dp0Drivers\STM32F4xx_HAL_Driver\Inc"

REM *** EDIT THIS: path to the Inc folder inside the unzipped STM32CubeF4 package ***
set "SOURCE_INC=C:\Users\Arshman Hassan\Desktop\STM32Cube_FW_F4_1.28.0\Drivers\STM32F4xx_HAL_Driver\Inc"

if not exist "%SOURCE_INC%\stm32f4xx_hal_adc.h" (
  echo SOURCE_INC not found or wrong path.
  echo Edit this .bat file and set SOURCE_INC to your unzipped package Inc folder.
  echo Example: STM32Cube_FW_F4_1.28.0\Drivers\STM32F4xx_HAL_Driver\Inc
  echo.
  echo Your project Inc folder is: %PROJECT_INC%
  echo See OPTION_B_EXACT_STEPS.md for manual copy steps.
  pause
  exit /b 1
)

echo Copying from: %SOURCE_INC%
echo          to:   %PROJECT_INC%
xcopy /Y /Q "%SOURCE_INC%\*.h" "%PROJECT_INC%\"
echo Done. Rebuild your project in STM32CubeIDE (Project -^> Clean, then Build).
pause
exit /b 0
