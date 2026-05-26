################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
DSP2803x_common/Uart.obj: ../DSP2803x_common/Uart.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 -O2 --fp_reassoc=off --include_path="D:/ti/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="F:/Users/LMD/workspace_v6_2/hf_motordrive/DSP2803x_common" --include_path="F:/Users/LMD/workspace_v6_2/hf_motordrive/DSP2803x_common/include" --include_path="F:/Users/LMD/workspace_v6_2/hf_motordrive/DSP2803x_headers/include" --advice:performance=all --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="DSP2803x_common/Uart.d" --obj_directory="DSP2803x_common" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


