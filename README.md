# gcan-boiler-plate
1. Base: Base that all branches should be able to start off that works and doesn't error, does not have gopher sense implemented
2. IO-Rev0: Implements Gopher Sense and sets up pins for testing Rev0 IO
    - IO-Validation: Small code changes for testing outputs upon board manufacturing completion
    - Input-Capture-Library: Code setting up Input Capture and DMA for transmission speed pulse sensor with Gopher Sense pulse_sensor.c library.
3. Structure-Changes: Add utils and change folder structure
4. Update-Variables: Add gcan variables and update in loops
    - Shifting: Adds over most shifitng code from 2022 repository, with updates for the new features/changes of 2023.
