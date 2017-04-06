#include <string>
#include <iostream>

#include "ROSNamesUtil.h"
#include <assert.h>

int main(int argc, char **argv)
{
  std::string s = "@:.$%^&*()[]!~`',-{}|=+;<>/abc_123\\\"";
  std::string r = toValidROSBaseName(s);
  std::cout << "toValidROSBaseName: " << s << " -> " << r << std::endl;
  assert( r ==    "___________________________abc_123__" ) ;
  std::cout << "\tok\n";
  r = withoutLeadingSlash("/hello/world");
  std::cout << "withoutLeadingSlash: /hello/world -> " << r << std::endl;
  assert(r == "hello/world");
  std::cout << "\tok\n";
  r = withoutLeadingSlash("hello world");
  std::cout << "withoutLeadingSlash: hello world -> " << r << std::endl;
  assert(r == "hello world");
  std::cout << "\tok\n";
  return 0;
}
