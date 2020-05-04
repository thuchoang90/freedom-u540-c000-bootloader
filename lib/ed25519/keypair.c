#include "ed25519.h"
#ifndef TEEHW
#include "sha3/sha3.h"
#else
#include "hwsha3.h"
#endif
#include "ge.h"


void ed25519_create_keypair(unsigned char *public_key, unsigned char *private_key, const unsigned char *seed) {
    ge_p3 A;

#ifndef TEEHW
    sha3(seed, 32, private_key, 64);
#else
    hwsha3_init();
    hwsha3_final(private_key, seed, 32);
#endif
    private_key[0] &= 248;
    private_key[31] &= 63;
    private_key[31] |= 64;

    ge_scalarmult_base(&A, private_key);
    ge_p3_tobytes(public_key, &A);
}
