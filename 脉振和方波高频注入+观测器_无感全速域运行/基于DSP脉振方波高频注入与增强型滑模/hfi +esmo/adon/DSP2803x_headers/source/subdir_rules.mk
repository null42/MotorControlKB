################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.obj: ../DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ti-cgt-c2000_16.9.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -Ooff --fp_reassoc=off --include_path="C:/ti/ti-cgt-c2000_16.9.0.LTS/include" --include_path="F:/FOC/HFI/TI_hfi +esmo/DSP2803x_common" --include_path="F:/FOC/HFI/TI_hfi +esmo/DSP2803x_common/include" --include_path="F:/FOC/HFI/TI_hfi +esmo/DSP2803x_headers/include" --advice:performance=all --define=_INLINE --define=VER="\"20211029_0856\"" --diag_warning=225 --diag_wrap=off --display_error_number --preproc_with_compile --preproc_dependency="DSP2803x_headers/source/DSP2803x_GlobalVariableDefs.pp" --obj_directory="DSP2803x_headers/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


