(cd drivers/net/wireless; \
ln -sf ../../../../802_11/madwifi/madwifi/ath ath; \
ln -sf ../../../../802_11/madwifi/madwifi/ath_hal _ath_hal;  \
ln -sf ../../../../802_11/madwifi/madwifi/ath_rate/onoe onoe; \
ln -sf ../../../../802_11/madwifi/ratectrl ratectrl; \
ln -sf ../../../../802_11/madwifi/madwifi/ath_phyerr ath_phyerr)
(cd drivers/net; \
ln -sf ../../../drivers/ar531x-enet ath; \
ln -sf ../../../drivers/idt-enet idt)
(cd net ; \
ln -sf ../../802_11/madwifi/madwifi/net80211 net80211)
