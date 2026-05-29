// 本代码说明如何修改 utility.c 以添加新信号
// 但实际中 signal_library → super_config.py → DATA_DETAILS 会自动处理此步骤

// 如果手动添加，在 write_data_to_file() 的 fprintf 中添加：
// fprintf(fw, DATA_FORMAT, DATA_DETAILS, OFSR.esoaf.xTL);
// 同时需要在 write_header_to_file() 的 DATA_LABELS 中添加列名
// 
// ★ 推荐：直接在 YAML signal_library 中添加，让 super_config.py 自动处理
