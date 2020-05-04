#include "ed25519.h"
#ifndef TEEHW
#include "sha3/sha3.h"
#else
#include "hwsha3.h"
#endif
#include "ge.h"
#include "sc.h"


void ed25519_sign(unsigned char *signature, const unsigned char *message, size_t message_len, const unsigned char *public_key, const unsigned char *private_key) {
#ifndef TEEHW
    sha3_ctx_t hash;
#endif
    unsigned char hram[64];
    unsigned char r[64];
    ge_p3 R;

#ifndef TEEHW
    sha3_init(&hash, 64);
    sha3_update(&hash, private_key + 32, 32);
    sha3_update(&hash, message, message_len);
    sha3_final(r, &hash);
#else
    hwsha3_init();
    hwsha3_update(private_key + 32, 32);
    hwsha3_final(r, message, message_len);
#endif

    sc_reduce(r);
    ge_scalarmult_base(&R, r);
    ge_p3_tobytes(signature, &R);

#ifndef TEEHW
    sha3_init(&hash, 64);
    sha3_update(&hash, signature, 32);
    sha3_update(&hash, public_key, 32);
    sha3_update(&hash, message, message_len);
    sha3_final(hram, &hash);
#else
    hwsha3_init();
    hwsha3_update(signature, 32);
    hwsha3_update(public_key, 32);
    hwsha3_final(hram, message, message_len);
#endif

    sc_reduce(hram);
    sc_muladd(signature + 32, hram, private_key, r);
}
