BIN="${1}.bin"
arm-none-eabi-size "${1}"
arm-none-eabi-objcopy -O binary -R .hot_init "${1}" "${BIN}"
pros ut
