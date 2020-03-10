// Get the "write" token in graphana topic in the manager
#define TOKEN "uFsX3b2hqmqpTYp3xbolE9eaDkpsmDwIlijtxyBsqS7qdtRZK3XBX9UaYH9GajwiSXq0JiPiXIBUvOmOcAJIOcr6OrW8zRbMx5bAGQsuwxtDBmVf61ITgJGXp"

// In the manager, the server is specified put it here
#define HOST "opentsdb.gra1.metrics.ovh.net"

// SHA1 fingerprint of the certificate
// Use this script by replacing <HOST> by the host you want to get the fingerprint
// OPENTSDB=<HOST>; echo | openssl s_client -showcerts -servername ${OPENTSDB} -connect ${OPENTSDB}:443 2>/dev/null | openssl x509 -noout -fingerprint -sha1 -inform pem | sed -e "s/.*=//g" | sed -e "s/\:/ /g"
#define FINGERPRINT "84 8F 74 12 8E 0D 25 E3 8C C7 47 8F 75 EF 30 15 70 03 A1 07 62 EB 75 AB BE 09 9B FF B2 66 4E F8"
