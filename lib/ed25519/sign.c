#include "ed25519.h"
#ifndef TEEHW
#include "sha3/sha3.h"
#else
#include "hwsha3.h"
#include "sifive/platform.h"
#endif
#include "ge.h"
#include "sc.h"


void ed25519_sign(unsigned char *signature, const unsigned char *message, size_t message_len, const unsigned char *public_key, const unsigned char *private_key) {
#ifndef TEEHW
    sha3_ctx_t hash;
    ge_p3 R;
#endif
    unsigned char hram[64];
    unsigned char r[64];

#ifndef TEEHW
    sha3_init(&hash, 64);
    sha3_update(&hash, private_key + 32, 32);
    sha3_update(&hash, message, message_len);
    sha3_final(r, &hash);
#else
    uint32_t *sig = (uint32_t*)signature;
    hwsha3_init();
    hwsha3_update(private_key + 32, 32);
    hwsha3_final(r, message, message_len);
#endif

    sc_reduce(r);

#ifndef TEEHW
    ge_scalarmult_base(&R, r);
    ge_p3_tobytes(signature, &R);

    sha3_init(&hash, 64);
    sha3_update(&hash, signature, 32);
    sha3_update(&hash, public_key, 32);
    sha3_update(&hash, message, message_len);
    sha3_final(hram, &hash);

    sc_reduce(hram);
    sc_muladd(signature + 32, hram, private_key, r);
#else
    for(int i = 0; i < 8; i++) {
        ED25519_REG(ED25519_REG_ADDR_K) = i;
        ED25519_REG(ED25519_REG_DATA_K) = *(((uint32_t*)(r)) + i);
    }
    ED25519_REG(ED25519_REG_STATUS) = 1; // Use the K memory
    while(!(ED25519_REG(ED25519_REG_STATUS) & 0x4)); // Wait
    for(int i = 0; i < 8; i++) {
        ED25519_REG(ED25519_REG_ADDR_QY) = i;
        sig[i] = ED25519_REG(ED25519_REG_DATA_QY);
    }

    // Calculate the H(R, A, M)
    hwsha3_init();
    hwsha3_update(signature, 32);
    hwsha3_update(public_key, 32);
    hwsha3_final(hram, message, message_len);

    // Calculate the S part with the addmult in hw
    for(int i = 0; i < 16; i++) {
        ED25519_REG(ED25519_REG_DATA_HKEY + i*4) = *(((uint32_t*)(private_key)) + i);
    }
    for(int i = 0; i < 16; i++) {
        ED25519_REG(ED25519_REG_DATA_HRAM + i*4) = *(((uint32_t*)(hram)) + i);
    }
    for(int i = 0; i < 16; i++) {
        ED25519_REG(ED25519_REG_DATA_HSM + i*4) = *(((uint32_t*)(r)) + i);
    }
    ED25519_REG(ED25519_REG_STATUS_3) = 1;
    while(!(ED25519_REG(ED25519_REG_STATUS_3) & 0x4)); // Wait
    for(int i = 0; i < 8; i++) {
        sig[i+8] = ED25519_REG(ED25519_REG_DATA_SIGN + i*4);
    }
#endif
}
