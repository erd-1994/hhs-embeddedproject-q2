#ifndef SERVER_ROOT_CERT_H
#define SERVER_ROOT_CERT_H

// Paste content of ca.crt here.
// Every line must end with \n" and start with "
static const char *server_root_cert_pem = "-----BEGIN CERTIFICATE-----\n"
                                          "<contents-of-ca.crt>\n"
                                          "-----END CERTIFICATE-----\n";
#endif
