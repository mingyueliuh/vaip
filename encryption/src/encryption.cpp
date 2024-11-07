/*
 *     The Xilinx Vitis AI Vaip in this distribution are provided under the
 * following free and permissive binary-only license, but are not provided in
 * source code form.  While the following free and permissive license is similar
 * to the BSD open source license, it is NOT the BSD open source license nor
 * other OSI-approved open source license.
 *
 *      Copyright (C) 2023 – 2024 Advanced Micro Devices, Inc. All rights
 * reserved.
 *
 *      Redistribution and use in binary form only, without modification, is
 * permitted provided that the following conditions are met:
 *
 *      1. Redistributions must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 *      2. The name of Xilinx, Inc. may not be used to endorse or promote
 * products redistributed with this software without specific prior written
 * permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY XILINX, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL XILINX, INC. BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *      PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

#include "encryption.hpp"
#include <memory>
#ifdef WITH_OPENSSL
#  include <openssl/aes.h>
#  include <openssl/conf.h>
#  include <openssl/err.h>
#  include <openssl/evp.h>
#endif
#include <stdexcept>

namespace vaip_encryption {
std::string aes_encryption(const std::string& str, const std::string& key) {
#ifdef WITH_OPENSSL
  if (key.size() * 8 != 256) {
    throw std::runtime_error("key size should be 256 bits");
  }
  EVP_CIPHER_CTX* ctx;
  if (!(ctx = EVP_CIPHER_CTX_new())) {
    throw std::runtime_error("encryption creating context failed");
  }

  if (1 != EVP_EncryptInit_ex(ctx, EVP_aes_256_ecb(), NULL,
                              (const unsigned char*)key.c_str(), NULL)) {
    throw std::runtime_error("encryption initialization failed");
  }

  std::string ciphertext;
  int len = 0;
  ciphertext.resize(str.size() + AES_BLOCK_SIZE);
  if (1 != EVP_EncryptUpdate(ctx, (unsigned char*)&ciphertext[0], &len,
                             (const unsigned char*)str.c_str(),
                             (int)str.length())) {
    throw std::runtime_error("encryption update failed");
  }
  int ciphertext_len = len;

  if (1 != EVP_EncryptFinal_ex(ctx, (unsigned char*)&ciphertext[len], &len)) {
    throw std::runtime_error("encryption finalization failed");
  }
  EVP_CIPHER_CTX_free(ctx);

  ciphertext_len += len;
  ciphertext.resize(ciphertext_len);
  return ciphertext;
#else
  return str;
#endif
}

std::string aes_decryption(const std::string& str, const std::string& key) {
#ifdef WITH_OPENSSL
  if (key.size() * 8 != 256) {
    throw std::runtime_error("key size should be 256 bits");
  }
  EVP_CIPHER_CTX* ctx;
  if (!(ctx = EVP_CIPHER_CTX_new())) {
    throw std::runtime_error("decryption creating context failed");
  }

  if (1 != EVP_DecryptInit_ex(ctx, EVP_aes_256_ecb(), NULL,
                              (const unsigned char*)key.c_str(), NULL)) {
    throw std::runtime_error("decryption initialization failed");
  }

  int len = 0;
  std::string plaintext;

  plaintext.resize(str.size());

  if (1 != EVP_DecryptUpdate(ctx, (unsigned char*)&plaintext[0], &len,
                             (const unsigned char*)str.c_str(),
                             (int)str.length())) {
    throw std::runtime_error("decryption update failed");
  }
  int plaintext_len = len;

  if (1 != EVP_DecryptFinal_ex(ctx, (unsigned char*)&plaintext[len], &len)) {
    throw std::runtime_error("decryption finalization failed");
  }
  EVP_CIPHER_CTX_free(ctx);

  plaintext_len += len;
  return plaintext.substr(0, plaintext_len);
#else
  return str;
#endif
}

} // namespace vaip_encryption