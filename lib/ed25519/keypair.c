#include "ed25519.h"
#ifndef TEEHW
#include "sha3/sha3.h"
#else
#include "hwsha3.h"
#include "sifive/platform.h"
#endif
#include "ge.h"


void ed25519_create_keypair(unsigned char *public_key, unsigned char *private_key, const unsigned char *seed) {
#ifndef TEEHW
    ge_p3 A;

    sha3(seed, 32, private_key, 64);
    private_key[0] &= 248;
    private_key[31] &= 63;
    private_key[31] |= 64;

    ge_scalarmult_base(&A, private_key);
    ge_p3_tobytes(public_key, &A);
#else
    //uint64_t *k = (uint64_t*)seed;
    uint32_t *pub = (uint32_t*)public_key;
    uint32_t *priv = (uint32_t*)private_key;

    hwsha3_init();
    hwsha3_final(private_key, seed, 32);

    for(int i = 0; i < 8; i++) {
        ED25519_REG(ED25519_REG_ADDR_K) = i;
        if(i == 0) // TODO: This is really necessary?
            ED25519_REG(ED25519_REG_DATA_K) = *(priv+i) & 0xFFFFFFF8;
        else if(i == 7)
            ED25519_REG(ED25519_REG_DATA_K) = (*(priv+i) & 0x3FFFFFFF) | 0x40000000;
        else
            ED25519_REG(ED25519_REG_DATA_K) = *(priv+i);
    }
    ED25519_REG(ED25519_REG_STATUS) = 1; // Use the K memory
    while(!(ED25519_REG(ED25519_REG_STATUS) & 0x4)); // Wait
    for(int i = 0; i < 8; i++) {
        ED25519_REG(ED25519_REG_ADDR_QY) = i;
        pub[i] = ED25519_REG(ED25519_REG_DATA_QY);
    }
#endif
}
