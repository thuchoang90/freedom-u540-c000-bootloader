#include "ed25519.h"
//#include "sha3/sha3.h"
#include "hwsha3.h"
#include "ge.h"
#include "sc.h"
#include "sifive/platform.h"

void ed25519_sign(unsigned char *signature, const unsigned char *message, size_t message_len, const unsigned char *public_key, const unsigned char *private_key) {
    unsigned char hram[64];
    unsigned char r[64];
    uint32_t *sig = (uint32_t*)signature;


    hwsha3_init();
    hwsha3_update(private_key + 32, 32);
    hwsha3_final(r, message, message_len);
    sc_reduce(r);
    
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
}
