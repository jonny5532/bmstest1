#pragma once

// Structs saved to NVM as raw bytes (drivers/chip/nvm.c) are defined between
// PERSIST_BEGIN and PERSIST_END. They keep the ordinary natural layout, but any
// padding the compiler would add is a build error (-Wpadded), which ensures
// that the struct is both aligned correctly and also effectively packed.
//
// This also ensures that the padding is zero, so can be reused for new fields
// later.
//
// Don't reorder, remove or retype existing fields though, without also
// bumping the struct's version (which we would generally avoid).
//
// An included struct (an anonymous member) has to be defined between them too,
// as -Wpadded only looks at the struct being defined.
#define PERSIST_BEGIN _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic error \"-Wpadded\"")
#define PERSIST_END _Pragma("GCC diagnostic pop")
