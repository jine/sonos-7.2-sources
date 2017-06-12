#
# Makefile for asymmetric cryptographic keys
#

obj-$(CONFIG_ASYMMETRIC_KEY_TYPE) += asymmetric_keys.o

asymmetric_keys-y := asymmetric_type.o signature.o

obj-$(CONFIG_ASYMMETRIC_PUBLIC_KEY_SUBTYPE) += public_key.o
obj-$(CONFIG_PUBLIC_KEY_ALGO_RSA) += rsa.o

#
# X.509 Certificate handling
#
obj-$(CONFIG_X509_CERTIFICATE_PARSER) += x509_key_parser.o
x509_key_parser-y := \
	x509-asn1.o \
	x509_rsakey-asn1.o \
	x509_cert_parser.o \
	x509_public_key.o

$(obj)/x509_cert_parser.o: $(obj)/x509-asn1.h $(obj)/x509_rsakey-asn1.h
$(obj)/x509-asn1.o: $(obj)/x509-asn1.c $(obj)/x509-asn1.h
$(obj)/x509_rsakey-asn1.o: $(obj)/x509_rsakey-asn1.c $(obj)/x509_rsakey-asn1.h

clean-files	+= x509-asn1.c x509-asn1.h
clean-files	+= x509_rsakey-asn1.c x509_rsakey-asn1.h

ifdef CONFIG_SONOS_SECBOOT
obj-y += sonos_signature_common_linux.o
obj-y += sonos_signature_keys.o
obj-y += sonos_signature_keys_$(SONOSBOARD).o
obj-y += sonos_signature_verify_linux.o

# rawrsa handling
obj-y += rawrsa_key_parser.o
rawrsa_key_parser-y := \
	rawrsa-asn1.o \
	rawrsa_parser.o \
	rawrsa_public_key.o

$(obj)/rawrsa_parser.o: $(obj)/rawrsa-asn1.h
$(obj)/rawrsa-asn1.o: $(obj)/rawrsa-asn1.c $(obj)/rawrsa-asn1.h

clean-files	+= rawrsa-asn1.c rawrsa-asn1.h

endif
