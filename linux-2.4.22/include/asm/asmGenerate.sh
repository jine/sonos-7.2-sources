#!/bin/sh

# run from within asm directory

cd ../asm-sh
for A in *; do
    file=../asm/$A
    echo "#if defined(__SONOS_LINUX_SH4__)" > $file;
    echo "#include \"asm-sh/$A\"" >> $file;
    echo "#elif defined(__SONOS_LINUX_PPC__)" >> $file;
    echo "#include \"asm-ppc/$A\"" >> $file;
    echo "#elif defined(SONOS_ARCH_ATTR_IS_EMULATOR)" >> $file;
    echo "#include \"asm-sh/$A\"" >> $file;
    echo "#else" >>$file;
    echo "#error must define either __SONOS_LINUX_[SH4/PPC/EMULATOR]__" >>$file;
    echo "#endif" >> $file;
done
cd ../asm-ppc
for A in *; do
    file=../asm/$A
    echo "#if defined(__SONOS_LINUX_SH4__)" > $file;
    echo "#include \"asm-sh/$A\"" >> $file;
    echo "#elif defined(__SONOS_LINUX_PPC__)" >> $file;
    echo "#include \"asm-ppc/$A\"" >> $file;
    echo "#elif defined(SONOS_ARCH_ATTR_IS_EMULATOR)" >> $file;
    echo "#include \"asm-sh/$A\"" >> $file;
    echo "#else" >>$file;
    echo "#error must define either __SONOS_LINUX_[SH4/PPC/EMULATOR]__" >>$file;
    echo "#endif" >> $file;
done
