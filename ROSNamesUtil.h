
#ifndef ROSNAMESUTIL_H
#define ROSNAMESUTIL_H


#include <string>
#include <algorithm>

bool valid_ros_base_name_char(char c)
{
  return (c == '_' || isalnum(c));
}


bool not_valid_ros_base_name_char(char c)
{
  return !valid_ros_base_name_char(c);
}

void makeValidROSBaseName(std::string& tn)
{
  std::replace_if(tn.begin(), tn.end(), not_valid_ros_base_name_char, '_');
}
  

std::string toValidROSBaseName(std::string tn)
{
  makeValidROSBaseName(tn);
  return tn;
}

std::string withoutLeadingSlash(std::string s)
{
  if( s.at(0) == '/' )
    return s.substr(1, s.size()-1);
  return s;
}

#endif
