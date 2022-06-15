#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <lighten/inet/address.hpp>

namespace lighten::inet {

address::address(const char *hostport)
{
  const char *port = strchr(hostport, ':');
  const char *host;
  if(port){
    int hlen = port - hostport;
    port++;
    char *p = (char *)alloca(hlen + 1);
    memcpy(p, hostport, hlen);
    p[hlen] = '\0';
    host = p;
  }
  else {
    host = hostport;
  }

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if(!inet_aton(host, &addr.sin_addr)){
    memset(&addr, 0, sizeof(addr));
    return;
  }

  if(port){
    char *ep;
    unsigned long ul = strtoul(port, &ep, 10);
    if(ep == port || *ep != '\0' || ul >= 65536){
      memset(&addr, 0, sizeof(addr));
      return;
    }
    addr.sin_port = htons((ul&0xffff));
  }
}

address::address(const char *host, int port)
{
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if(!inet_aton(host, &addr.sin_addr)){
    memset(&addr, 0, sizeof(addr));
    return;
  }

  if(port < 0 || port >= 65536){
    memset(&addr, 0, sizeof(addr));
    return;
  }
  addr.sin_port = htons(port);
}

address::address(int port)
{
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;

  if(port < 0 || port >= 65536){
    memset(&addr, 0, sizeof(addr));
    return;
  }
  addr.sin_port = htons(port);
}

}
