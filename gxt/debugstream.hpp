/**
 * @file debugstream.h
 * @author gxt_kt (gxt_kt@163.com)
 * @brief  If you have use the qDebug() function of "Qt" before, you must use this module easily.
 * And the qDebug is change name to gDebug here. The detail see the "@attention".
 * The github address is https://github.com/gxt-kt/gDebugV2.0
 * @version 0.72
 * @date 2023-08-2
 *
 * @copyright Copyright (c) 2022
 *
 * @attention You can use the default gDebug() function to output the debug stream.
 * Such as gDebug("hello world"); , Complexly you can write like gDebug("hello") << "world"; and so on.
 * And the default gDebug() has enable the space and newline.
 * If you use the class DebugStream create a new instantiation. The space funciton is exist but the
 * auto newline is invalid.
 */

#ifndef DEBUGSTREAM_H__
#define DEBUGSTREAM_H__


#include <string>
#include <functional>
#include <memory>
#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <sstream>

// Reference : https://github.com/p-ranav/pprint
#define PPRINT_EN 1

// There will no debug stream output if define the NO_DEBUG_OUTPUT
// #define NO_DEBUG_OUTPUT




#if PPRINT_EN
#include <iostream>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <vector>
#include <list>
#include <deque>
#include <set>
#include <unordered_set>
#include <array>
#include <map>
#include <unordered_map>
#include <iomanip>
#include <variant>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iosfwd>
#include <limits>
#include <string_view>
#include <optional>
#include <utility>
#include <sstream>
#include <queue>
#include <stack>
#include <tuple>
#include <initializer_list>
#include <complex>
#include <cmath>
#include <memory>
#ifdef __GNUG__
#include <cstdlib>
#include <memory>
#include <cxxabi.h>
#endif
#endif

namespace gxt {

#if PPRINT_EN

// support c++17 variant and optional
#define SUPPORTS_CPP17 (__cplusplus >= 201703L)

// Check if a type is stream writable, i.e., std::cout << foo;
template<typename S, typename T, typename = void>
struct is_to_stream_writable: std::false_type {};

template<typename S, typename T>
struct is_to_stream_writable<S, T,
           std::void_t<  decltype( std::declval<S&>()<<std::declval<T>() ) >>
  : std::true_type {};


// Printing std::tuple
// The indices trick: http://loungecpp.wikidot.com/tips-and-tricks:indices
namespace pprint {

  template<std::size_t...> struct seq{};

  template<std::size_t N, std::size_t... Is>
  struct gen_seq : gen_seq<N-1, N-1, Is...>{};

  template<std::size_t... Is>
  struct gen_seq<0, Is...> : seq<Is...>{};

  template<typename T>
  inline T to_string(T value) {
    return value;
  }

  inline std::string to_string(char value) {
    return "'" + std::string(1, value) + "'";
  }

  inline std::string to_string(const char * value) {
    return "\"" + std::string(value) + "\"";
  }

  inline std::string to_string(const std::string& value) {
    return "\"" + value + "\"";
  }

  template<class Ch, class Tr, class Tuple, std::size_t... Is>
  void print_tuple(std::basic_ostream<Ch,Tr>& os, Tuple const& t, seq<Is...>){
    using swallow = int[];
    (void)swallow{0, (void(os << (Is == 0? "" : ", ") << to_string(std::get<Is>(t))), 0)...};
  }

}

template<class Ch, class Tr, class... Args>
auto operator<<(std::basic_ostream<Ch, Tr>& os, std::tuple<Args...> const& t)
  -> std::basic_ostream<Ch, Tr>& {
  os << "(";
  pprint::print_tuple(os, t, pprint::gen_seq<sizeof...(Args)>());
  return os << ")";
}

namespace pprint {

  // Some utility structs to check template specialization
  template<typename Test, template<typename...> class Ref>
  struct is_specialization : std::false_type {};

  template<template<typename...> class Ref, typename... Args>
  struct is_specialization<Ref<Args...>, Ref> : std::true_type {};

  template<typename ...>
  using to_void = void;

  template<typename T, typename = void>
  struct is_container : std::false_type
  {};

  template<typename T>
  struct is_container<T,
          to_void<decltype(std::declval<T>().begin()),
              decltype(std::declval<T>().end()),
              typename T::value_type
          >> : std::true_type // will  be enabled for iterable objects
  {};

  class PrettyPrinter {
  private:
    std::ostream& stream_;
    std::string line_terminator_;
    size_t indent_;
    bool quotes_;
    bool compact_;

  public:

    PrettyPrinter(std::ostream& stream = std::cout) :
      stream_(stream),
      line_terminator_("\n"),
      indent_(2),
      quotes_(false),
      compact_(false) {}

    PrettyPrinter& line_terminator(const std::string& value) {
      line_terminator_ = value;
      return *this;
    }

    PrettyPrinter& indent(size_t indent) {
      indent_ = indent;
      return *this;
    }

    PrettyPrinter& compact(bool value) {
      compact_ = value;
      return *this;
    }

    PrettyPrinter& quotes(bool value) {
      quotes_ = value;
      return *this;
    }

    template <typename T>
    void print(T value) {
      print_internal(value, 0, line_terminator_, 0);
    }

    template <typename T>
    void print(std::initializer_list<T> value) {
      print_internal(value, 0, line_terminator_, 0);
    }

    template<typename T, typename... Targs>
    void print(T value, Targs... Fargs) {
      print_internal(value, 0, "", 0);
      auto current_quotes = quotes_;
      quotes_ = false;
      print_internal(" ", 0, "", 0);
      quotes_ = current_quotes;
      print(Fargs...);
    }

    template <typename T>
    void print_inline(T value) {
      print_internal(value, indent_, "", 0);
    }

    template <typename T>
    void print_inline(std::initializer_list<T> value) {
      print_internal(value, indent_, "", 0);
    }

    template<typename T, typename... Targs>
    void print_inline(T value, Targs... Fargs) {
      print_internal(value, indent_, "", 0);
      auto current_quotes = quotes_;
      quotes_ = false;
      print_internal(" ", 0, "", 0);
      quotes_ = current_quotes;
      print_inline(Fargs...);
    }

  private:

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value == true, void>::type
    print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    template <typename T>
    typename std::enable_if<std::is_null_pointer<T>::value == true, void>::type
    print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << "nullptr" << line_terminator;
    }

    void print_internal(float value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << 'f' << line_terminator;
    }

    void print_internal(double value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    void print_internal(const std::string& value, size_t indent = 0, const std::string& line_terminator = "\n",
      size_t level = 0) {
      if (!quotes_)
        print_internal_without_quotes(value, indent, line_terminator, level);
      else
        stream_ << std::string(indent, ' ') << "\"" << value << "\"" << line_terminator;
    }

    void print_internal(const char * value, size_t indent = 0, const std::string& line_terminator = "\n",
      size_t level = 0) {
      if (!quotes_)
        print_internal_without_quotes(value, indent, line_terminator, level);
      else
        stream_ << std::string(indent, ' ') << "\"" << value << "\"" << line_terminator;
    }

    void print_internal(char value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      if (!quotes_)
        print_internal_without_quotes(value, indent, line_terminator, level);
      else
        stream_ << std::string(indent, ' ') << "'" << value << "'" << line_terminator;
    }

    void print_internal_without_quotes(const std::string& value, size_t indent = 0,
      const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    void print_internal_without_quotes(const char * value, size_t indent = 0,
      const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    void print_internal_without_quotes(char value, size_t indent = 0, const std::string& line_terminator = "\n",
      size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    void print_internal(bool value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << (value ? "true" : "false") << line_terminator;
    }

    template <typename T>
    typename std::enable_if<std::is_pointer<T>::value == true, void>::type
    print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      if (value == nullptr) {
        return print_internal(nullptr, indent, line_terminator, level);
      }
      stream_ << std::string(indent, ' ') << "<" << type(value) << " at "
              << value << ">" << line_terminator;
    }

    std::string demangle(const char* name) {
#ifdef __GNUG__
      int status = -4;
      std::unique_ptr<char, void(*)(void*)> res {
        abi::__cxa_demangle(name, NULL, NULL, &status),
        std::free
      };
      return (status==0) ? res.get() : name;
#else
      return name;
#endif
    }

    template <class T>
    std::string type(const T& t) {
      return demangle(typeid(t).name());
    }

    // TODO : add new magic_enum impl function ( require : not need c++17 )
    //
    // template <typename T>
    // typename std::enable_if<std::is_enum<T>::value == true, void>::type
    // print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
    //   auto enum_string = magic_enum::enum_name(value);
    //   if (enum_string.has_value()) {
    //     stream_ << std::string(indent, ' ') << enum_string.value()
    //             << line_terminator;
    //   }
    //   else {
    //     stream_ << std::string(indent, ' ') << static_cast<std::underlying_type_t<T>>(value)
    //             << line_terminator;
    //   }
    // }

    template <typename T>
    typename std::enable_if<std::is_class<T>::value == true &&
        is_to_stream_writable<std::ostream, T>::value == true &&
        std::is_enum<T>::value == false &&
        is_specialization<T, std::unique_ptr>::value == false &&
        is_specialization<T, std::shared_ptr>::value == false &&
        is_specialization<T, std::weak_ptr>::value == false &&
        is_specialization<T, std::tuple>::value == false &&
        #if SUPPORTS_CPP17
        is_specialization<T, std::variant>::value == false &&
        #endif
        is_specialization<T, std::vector>::value == false &&
        is_specialization<T, std::list>::value == false &&
        is_specialization<T, std::deque>::value == false &&
        is_specialization<T, std::queue>::value == false &&
        is_specialization<T, std::priority_queue>::value == false &&
        is_specialization<T, std::stack>::value == false &&
        is_specialization<T, std::set>::value == false &&
        is_specialization<T, std::multiset>::value == false &&
        is_specialization<T, std::unordered_set>::value == false &&
        is_specialization<T, std::unordered_multiset>::value == false &&
        is_specialization<T, std::map>::value == false &&
        is_specialization<T, std::multimap>::value == false &&
        is_specialization<T, std::unordered_map>::value == false &&
        is_specialization<T, std::unordered_multimap>::value == false, void>::type
    print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << value << line_terminator;
    }

    template <typename T>
    typename std::enable_if<std::is_class<T>::value == true &&
            is_to_stream_writable<std::ostream, T>::value == false &&
            std::is_enum<T>::value == false &&
            is_specialization<T, std::unique_ptr>::value == false &&
            is_specialization<T, std::shared_ptr>::value == false &&
            is_specialization<T, std::weak_ptr>::value == false &&
            is_specialization<T, std::tuple>::value == false &&
            #if SUPPORTS_CPP17
            is_specialization<T, std::variant>::value == false &&
            #endif
            is_specialization<T, std::vector>::value == false &&
            is_specialization<T, std::list>::value == false &&
            is_specialization<T, std::deque>::value == false &&
            is_specialization<T, std::queue>::value == false &&
            is_specialization<T, std::priority_queue>::value == false &&
            is_specialization<T, std::stack>::value == false &&
            is_specialization<T, std::set>::value == false &&
            is_specialization<T, std::multiset>::value == false &&
            is_specialization<T, std::unordered_set>::value == false &&
            is_specialization<T, std::unordered_multiset>::value == false &&
            is_specialization<T, std::map>::value == false &&
            is_specialization<T, std::multimap>::value == false &&
            is_specialization<T, std::unordered_map>::value == false &&
            is_specialization<T, std::unordered_multimap>::value == false, void>::type
            print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << "<Object " << type(value) << ">"
              << line_terminator;
    }

    template <typename T>
    typename std::enable_if<std::is_member_function_pointer<T>::value == true, void>::type
    print_internal(T value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      stream_ << std::string(indent, ' ') << "<Object.method " << type(value)
              << " at " << &value << ">"
              << line_terminator;
    }

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::vector>::value, void>::type
            print_internal(const Container& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      typedef typename Container::value_type T;
      if (level == 0 && !compact_) {
        if (value.size() == 0) {
          print_internal_without_quotes("[", 0, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", 0, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", 0, "\n");
          print_internal(value.front(), indent + indent_, "", level + 1);
          if (value.size() > 1 && is_container<T>::value == false)
            print_internal_without_quotes(", ", 0, "\n");
          else if (is_container<T>::value)
            print_internal_without_quotes(", ", 0, "\n");
          for (size_t i = 1; i < value.size() - 1; i++) {
            print_internal(value[i], indent + indent_, "", level + 1);
            if (is_container<T>::value == false)
              print_internal_without_quotes(", ", 0, "\n");
            else
              print_internal_without_quotes(", ", 0, "\n");
          }
          if (value.size() > 1) {
            print_internal(value.back(), indent + indent_, "\n", level + 1);
          }
        }
        if (value.size() == 0)
          print_internal_without_quotes("]", indent, "");
        else if (is_container<T>::value == false)
          print_internal_without_quotes("]", indent, "");
        else
          print_internal_without_quotes(line_terminator_ + "]", indent, "");
        print_internal_without_quotes(line_terminator_, 0, "");
      }
      else {
        if (value.size() == 0) {
          print_internal_without_quotes("[", indent, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
          if (value.size() > 1)
            print_internal_without_quotes(", ", 0, "");
          for (size_t i = 1; i < value.size() - 1; i++) {
            print_internal(value[i], 0, "", level + 1);
            print_internal_without_quotes(", ", 0, "");
          }
          if (value.size() > 1) {
            print_internal(value.back(), 0, "", level + 1);
          }
        }
        print_internal_without_quotes("]", 0, "");
        if (level == 0 && compact_)
          print_internal_without_quotes(line_terminator_, 0, "");
      }

    }

    template <typename T, unsigned long int S>
    void print_internal(const std::array<T, S>& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      if (level == 0 && !compact_) {
        if (value.size() == 0) {
          print_internal_without_quotes("[", 0, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", 0, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", 0, "\n");
          print_internal(value.front(), indent + indent_, "", level + 1);
          if (value.size() > 1 && is_container<T>::value == false)
            print_internal_without_quotes(", ", 0, "\n");
          else if (is_container<T>::value)
            print_internal_without_quotes(", ", 0, "\n");
          for (size_t i = 1; i < value.size() - 1; i++) {
            print_internal(value[i], indent + indent_, "", level + 1);
            if (is_container<T>::value == false)
              print_internal_without_quotes(", ", 0, "\n");
            else
              print_internal_without_quotes(", ", 0, "\n");
          }
          if (value.size() > 1) {
            print_internal(value.back(), indent + indent_, "\n", level + 1);
          }
        }
        if (value.size() == 0)
          print_internal_without_quotes("]", indent, "");
        else if (is_container<T>::value == false)
          print_internal_without_quotes("]", indent, "");
        else
          print_internal_without_quotes(line_terminator_ + "]", indent, "");
        print_internal_without_quotes(line_terminator_, 0, "");
      }
      else {
        if (value.size() == 0) {
          print_internal_without_quotes("[", indent, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
          if (value.size() > 1)
            print_internal_without_quotes(", ", 0, "");
          for (size_t i = 1; i < value.size() - 1; i++) {
            print_internal(value[i], 0, "", level + 1);
            print_internal_without_quotes(", ", 0, "");
          }
          if (value.size() > 1) {
            print_internal(value.back(), 0, "", level + 1);
          }
        }
        print_internal_without_quotes("]", 0, "");
        if (level == 0 && compact_)
          print_internal_without_quotes(line_terminator_, 0, "");
      }

    }

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::list>::value ||
            is_specialization<Container, std::deque>::value,
            void>::type print_internal(const Container& value, size_t indent = 0,
            const std::string& line_terminator = "\n",
            size_t level = 0) {
      typedef typename Container::value_type T;
      if (level == 0 && !compact_) {
        if (value.size() == 0) {
          print_internal_without_quotes("[", 0, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", 0, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", 0, "\n");
          print_internal(value.front(), indent + indent_, "", level + 1);
          if (value.size() > 1 && is_container<T>::value == false)
            print_internal_without_quotes(", ", 0, "\n");
          else if (is_container<T>::value)
            print_internal_without_quotes(", ", 0, "\n");

          typename Container::const_iterator iterator;
          for (iterator = std::next(value.begin()); iterator != std::prev(value.end()); ++iterator) {
            print_internal(*iterator, indent + indent_, "", level + 1);
            if (is_container<T>::value == false)
              print_internal_without_quotes(", ", 0, "\n");
            else
              print_internal_without_quotes(", ", 0, "\n");
          }

          if (value.size() > 1) {
            print_internal(value.back(), indent + indent_, "\n", level + 1);
          }
        }
        if (value.size() == 0)
          print_internal_without_quotes("]", indent, "");
        else if (is_container<T>::value == false)
          print_internal_without_quotes("]", indent, "");
        else
          print_internal_without_quotes(line_terminator_ + "]", indent, "");
        print_internal_without_quotes(line_terminator_, 0, "");
      }
      else {
        if (value.size() == 0) {
          print_internal_without_quotes("[", indent, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
        }
        else if (value.size() > 0) {
          print_internal_without_quotes("[", indent, "");
          print_internal(value.front(), 0, "", level + 1);
          if (value.size() > 1)
            print_internal_without_quotes(", ", 0, "");

          typename Container::const_iterator iterator;
          for (iterator = std::next(value.begin()); iterator != std::prev(value.end()); ++iterator) {
            print_internal(*iterator, 0, "", level + 1);
            print_internal_without_quotes(", ", 0, "");
          }

          if (value.size() > 1) {
            print_internal(value.back(), 0, "", level + 1);
          }
        }
        print_internal_without_quotes("]", 0, "");
        if (level == 0 && compact_)
          print_internal_without_quotes(line_terminator_, 0, "");
      }

    }

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::set>::value ||
            is_specialization<Container, std::multiset>::value ||
            is_specialization<Container, std::unordered_set>::value ||
            is_specialization<Container, std::unordered_multiset>::value, void>::type
            print_internal(const Container& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      typedef typename Container::value_type T;
      if (level == 0 && !compact_) {
        if (value.size() == 0) {
          print_internal_without_quotes("{", 0, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("{", 0, "");
          print_internal(*(value.begin()), 0, "", level + 1);
        }
        else {
          print_internal_without_quotes("{", 0, "\n");
          print_internal(*(value.begin()), indent + indent_, "", level + 1);
          if (value.size() > 1 && is_container<T>::value == false)
            print_internal_without_quotes(", ", 0, "\n");
          else if (is_container<T>::value)
            print_internal_without_quotes(", ", 0, "\n");

          typename Container::const_iterator iterator;
          for (iterator = std::next(value.begin()); (iterator != value.end()) && (std::next(iterator) != value.end()); ++iterator) {
            print_internal(*iterator, indent + indent_, "", level + 1);
            if (is_container<T>::value == false)
              print_internal_without_quotes(", ", 0, "\n");
            else
              print_internal_without_quotes(", ", 0, "\n");
          }

          if (value.size() > 1) {
            print_internal(*iterator, indent + indent_, "\n", level + 1);
          }
        }
        if (value.size() == 0)
          print_internal_without_quotes("}", indent, "");
        else if (is_container<T>::value == false)
          print_internal_without_quotes("}", indent, "");
        else
          print_internal_without_quotes(line_terminator_ + "}", indent, "");
        print_internal_without_quotes(line_terminator_, 0, "");
      }
      else {
        if (value.size() == 0) {
          print_internal_without_quotes("{", indent, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("{", indent, "");
          print_internal(*(value.begin()), 0, "", level + 1);
        }
        else {
          print_internal_without_quotes("{", indent, "");
          print_internal(*(value.begin()), 0, "", level + 1);
          if (value.size() > 1)
            print_internal_without_quotes(", ", 0, "");

          typename Container::const_iterator iterator;
          for (iterator = std::next(value.begin()); (iterator != value.end()) && (std::next(iterator) != value.end()); ++iterator) {
            print_internal(*iterator, 0, "", level + 1);
            print_internal_without_quotes(", ", 0, "");
          }

          if (value.size() > 1) {
            print_internal(*iterator, 0, "", level + 1);
          }
        }
        print_internal_without_quotes("}", 0, "");
        if (level == 0 && compact_)
          print_internal_without_quotes(line_terminator_, 0, "");
      }

    }

    template <typename T>
    typename std::enable_if<is_specialization<T, std::map>::value == true ||
            is_specialization<T, std::multimap>::value == true ||
            is_specialization<T, std::unordered_map>::value == true ||
            is_specialization<T, std::unordered_multimap>::value == true, void>::type
            print_internal(const T& value, size_t indent = 0, const std::string& line_terminator = "\n", size_t level = 0) {
      typedef typename T::mapped_type Value;
      if (level == 0 && !compact_) {
        if (value.size() == 0) {
          print_internal_without_quotes("{", 0, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("{", 0, "");
          for (auto& kvpair : value) {
            print_internal(kvpair.first, 0, "", level + 1);
            print_internal_without_quotes(" : ", 0, "");
            print_internal(kvpair.second, 0, "", level + 1);
          }
        }
        else if (value.size() > 0) {
          size_t count = 0;
          for (auto& kvpair : value) {
            if (count == 0) {
              print_internal_without_quotes("{", 0, "\n");
              print_internal(kvpair.first, indent + indent_, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "", level + 1);
              if (value.size() > 1 && is_container<Value>::value == false)
                print_internal_without_quotes(", ", 0, "\n");
              else if (is_container<Value>::value)
                print_internal_without_quotes(", ", 0, "\n");
            }
            else if (count + 1 < value.size()) {
              print_internal(kvpair.first, indent + indent_, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "", level + 1);
              if (is_container<Value>::value == false)
                print_internal_without_quotes(", ", 0, "\n");
              else
                print_internal_without_quotes(", ", 0, "\n");
            }
            else {
              print_internal(kvpair.first, indent + indent_, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "\n", level + 1);
            }
            count += 1;
          }
        }
        if (value.size() == 0)
          print_internal_without_quotes("}", indent, "");
        else if (is_container<Value>::value == false)
          print_internal_without_quotes("}", indent, "");
        else
          print_internal_without_quotes(line_terminator_ + "}", indent, "");
        print_internal_without_quotes(line_terminator_, 0, "");
      }

      else {
        if (value.size() == 0) {
          print_internal_without_quotes("{", indent, "");
        }
        else if (value.size() == 1) {
          print_internal_without_quotes("{", indent, "");
          for (auto& kvpair : value) {
            print_internal(kvpair.first, 0, "", level + 1);
            print_internal_without_quotes(" : ", 0, "");
            print_internal(kvpair.second, 0, "", level + 1);
          }
        }
        else if (value.size() > 0) {
          size_t count = 0;
          for (auto& kvpair : value) {
            if (count == 0) {
              print_internal_without_quotes("{", indent, "");
              print_internal(kvpair.first, 0, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "", level + 1);
              print_internal_without_quotes(", ", 0, "");
            }
            else if (count + 1 < value.size()) {
              print_internal(kvpair.first, indent + indent_, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "", level + 1);
              print_internal_without_quotes(", ", 0, "");
            }
            else {
              print_internal(kvpair.first, 0, "", level + 1);
              print_internal_without_quotes(" : ", 0, "");
              print_internal(kvpair.second, 0, "", level + 1);
            }
            count += 1;
          }
        }
        print_internal_without_quotes("}", 0, "");
        if (level == 0 && compact_)
          print_internal_without_quotes(line_terminator_, 0, "");
      }
    }

    template <typename Key, typename Value>
    void print_internal(std::pair<Key, Value> value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      print_internal_without_quotes("(", indent, "");
      print_internal(value.first, 0, "");
      print_internal_without_quotes(", ", 0, "");
      print_internal(value.second, 0, "");
      print_internal_without_quotes(")", 0, line_terminator, level);
    }

    #if SUPPORTS_CPP17
    template <class ...Ts>
    void print_internal(std::variant<Ts...> value, size_t indent = 0,
        const std::string& line_terminator = "\n", size_t level = 0) {
      std::visit([=](const auto& value) { print_internal(value, indent, line_terminator, level); }, value);
    }

    template <typename T>
    void print_internal(std::optional<T> value, size_t indent = 0,
        const std::string& line_terminator = "\n", size_t level = 0) {
      if (value) {
        print_internal(value.value(), indent, line_terminator, level);
      }
      else {
        print_internal_without_quotes("nullopt", indent, line_terminator, level);
      }
    }
    #endif

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::queue>::value, void>::type
            print_internal(const Container& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      auto current_compact = compact_;
      compact_ = true;
      typedef typename Container::value_type T;
      auto local = value;
      std::vector<T> local_vector;
      while (!local.empty()) {
        local_vector.push_back(local.front());
        local.pop();
      }
      print_internal(local_vector, indent, line_terminator, level);
      compact_ = current_compact;
    }

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::priority_queue>::value, void>::type
    print_internal(const Container& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      auto current_compact = compact_;
      compact_ = true;
      typedef typename Container::value_type T;
      auto local = value;
      std::vector<T> local_vector;
      while (!local.empty()) {
        local_vector.push_back(local.top());
        local.pop();
      }
      print_internal(local_vector, indent, line_terminator, level);
      compact_ = current_compact;
    }

    template <typename T>
    void print_internal(std::initializer_list<T> value, size_t indent = 0,
            const std::string& line_terminator = "\n", size_t level = 0) {
      std::multiset<T> local;
      for(const T& x : value) {
        local.insert(x);
      }
      print_internal(local, indent, line_terminator_, level);
    }

    template <typename Container>
    typename std::enable_if<is_specialization<Container, std::stack>::value, void>::type
    print_internal(const Container& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      bool current_compact = compact_;
      compact_ = false; // Need to print a stack like its a stack, i.e., vertical
      typedef typename Container::value_type T;
      auto local = value;
      std::vector<T> local_vector;
      while (!local.empty()) {
        local_vector.push_back(local.top());
        local.pop();
      }
      print_internal(local_vector, indent, line_terminator, level);
      compact_ = current_compact;
    }

    template<class... Args>
    void print_internal(const std::tuple<Args...>& value, size_t indent = 0, const std::string& line_terminator = "\n",
            size_t level = 0) {
      stream_ << std::string(indent, ' ') << value
              << line_terminator;
    }

    template<typename T>
    void print_internal(const std::complex<T>& value, size_t indent = 0, const std::string& line_terminator = "\n",
        size_t level = 0) {
      stream_ << std::string(indent, ' ') << "(" <<
      value.real() << " + " << value.imag() << "i)"
                   << line_terminator;
    }

    template<typename Pointer>
    typename std::enable_if<is_specialization<Pointer, std::unique_ptr>::value ||
        is_specialization<Pointer, std::shared_ptr>::value ||
        is_specialization<Pointer, std::weak_ptr>::value, void>::type
        print_internal(const Pointer& value, size_t indent = 0, const std::string& line_terminator = "\n",
        size_t level = 0) {
      stream_ << std::string(indent, ' ') << "<" <<
      type(value) << " at " << &value << ">"
                  << line_terminator;
    }

  };

}
#endif

// The default call back function : use the printf to send the stream
inline void DebugSendStringCallBack_Default_(const char *str, int num) {
  printf("%.*s",num,str);
}

// Set the endl compatible with std::endl;
class DebugStreamEndl {};
inline void endl(DebugStreamEndl){};

//--------------------------------------------------
static const char* DEBUG_STREAM_COLOR_FG_NORMAL = "\x1B[0m";
static const char* DEBUG_STREAM_COLOR_FG_BLACK  = "\x1B[30m";
static const char* DEBUG_STREAM_COLOR_FG_RED    = "\x1B[31m";
static const char* DEBUG_STREAM_COLOR_FG_GREEN  = "\x1B[32m";
static const char* DEBUG_STREAM_COLOR_FG_YELLOW = "\x1B[33m";
static const char* DEBUG_STREAM_COLOR_FG_BLUE   = "\x1B[34m";
static const char* DEBUG_STREAM_COLOR_FG_MAGENTA= "\x1B[35m";
static const char* DEBUG_STREAM_COLOR_FG_CYAN   = "\x1B[36m";
static const char* DEBUG_STREAM_COLOR_FG_WHITE  = "\x1B[37m";
//--------------------------------------------------
static const char* DEBUG_STREAM_COLOR_BG_NORMAL = "\x1B[49m";
static const char* DEBUG_STREAM_COLOR_BG_BLACK  = "\x1b[40m";
static const char* DEBUG_STREAM_COLOR_BG_RED    = "\x1b[41m";
static const char* DEBUG_STREAM_COLOR_BG_GREEN  = "\x1B[42m";
static const char* DEBUG_STREAM_COLOR_BG_YELLOW = "\x1B[43m";
static const char* DEBUG_STREAM_COLOR_BG_BLUE   = "\x1B[44m";
static const char* DEBUG_STREAM_COLOR_BG_MAGENTA= "\x1B[45m";
static const char* DEBUG_STREAM_COLOR_BG_CYAN   = "\x1B[46m";
static const char* DEBUG_STREAM_COLOR_BG_WHITE  = "\x1B[47m";
//--------------------------------------------------
class normal_fg_t {}; inline void normal_fg (normal_fg_t ) {}
class black_fg_t  {}; inline void black_fg  (black_fg_t  ) {}
class red_fg_t    {}; inline void red_fg    (red_fg_t    ) {}
class green_fg_t  {}; inline void green_fg  (green_fg_t  ) {}
class yellow_fg_t {}; inline void yellow_fg (yellow_fg_t ) {}
class blue_fg_t   {}; inline void blue_fg   (blue_fg_t   ) {}
class magenta_fg_t{}; inline void magenta_fg(magenta_fg_t) {}
class cyan_fg_t   {}; inline void cyan_fg   (cyan_fg_t   ) {}
class white_fg_t  {}; inline void white_fg  (white_fg_t  ) {}
//--------------------------------------------------
class normal_bg_t {}; inline void normal_bg (normal_bg_t ) {}
class black_bg_t  {}; inline void black_bg  (black_bg_t  ) {}
class red_bg_t    {}; inline void red_bg    (red_bg_t    ) {}
class green_bg_t  {}; inline void green_bg  (green_bg_t  ) {}
class yellow_bg_t {}; inline void yellow_bg (yellow_bg_t ) {}
class blue_bg_t   {}; inline void blue_bg   (blue_bg_t   ) {}
class magenta_bg_t{}; inline void magenta_bg(magenta_bg_t) {}
class cyan_bg_t   {}; inline void cyan_bg   (cyan_bg_t   ) {}
class white_bg_t  {}; inline void white_bg  (white_bg_t  ) {}

// set the debug color
class debug_general_t     {}; inline void GENERAL     (debug_general_t)     {}
class debug_status_t      {}; inline void STATUS      (debug_status_t)      {}
class debug_warning_t     {}; inline void WARNING     (debug_warning_t)     {}
class debug_error_t       {}; inline void ERROR       (debug_error_t)       {}
class debug_fatal_error_t {}; inline void FATAL_ERROR (debug_fatal_error_t) {}


class DebugStream : 
      public std::stringstream
#if PPRINT_EN
, public pprint::PrettyPrinter
#endif
{
 public:
  //======================================
  explicit DebugStream \
  (std::function<void(const char*,int)>  fun_ = DebugSendStringCallBack_Default_, \
  int buf_len_ = 256);
  DebugStream(const DebugStream &obj);
  DebugStream(DebugStream &&obj);
  DebugStream& operator=(const DebugStream &obj);
  DebugStream& operator=(DebugStream &&obj);
  ~DebugStream();
  //===============================================================
  inline DebugStream &OutEn(bool en)    {out_en=en;       return *this;}
  inline DebugStream &NoNewLine()       {newline=false;   return *this;}
  inline DebugStream &NewLine()         {newline=true;    return *this;}
  inline DebugStream &Space()           {space=true;      return *this;}
  inline DebugStream &NoSpace()         {space=false;     return *this;}
  inline DebugStream &Terminate()       {terminate=true;  return *this;}
  inline DebugStream &ClearColor()      {clear_color=true;return *this;}
  //===============================================================

  //=========================================
  // inline DebugStream &print()                                    {return *this;}
  inline DebugStream &operator<<(const DebugStream& stream)      {newline=false;return MayBeSpace();}
  inline DebugStream &operator<<(void(*)(DebugStreamEndl)) {(*this)<<"\n"; return *this;}

  inline DebugStream &operator<<(void(*)(normal_fg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_NORMAL; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(black_fg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_BLACK; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(red_fg_t))    {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_RED; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(green_fg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_GREEN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(yellow_fg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_YELLOW; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(blue_fg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_BLUE; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(magenta_fg_t)){(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_MAGENTA; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(cyan_fg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_CYAN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(white_fg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_FG_WHITE; return (*this).Space();}

  inline DebugStream &operator<<(void(*)(normal_bg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_NORMAL; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(black_bg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_BLACK; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(red_bg_t))    {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_RED; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(green_bg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_GREEN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(yellow_bg_t)) {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_YELLOW; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(blue_bg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_BLUE; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(magenta_bg_t)){(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_MAGENTA; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(cyan_bg_t))   {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_CYAN; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(white_bg_t))  {(*this).NoSpace()<<DEBUG_STREAM_COLOR_BG_WHITE; return (*this).Space();}

  inline DebugStream &operator<<(void(*)(debug_general_t))     {(*this).NoSpace()<<normal_fg<<normal_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_status_t))      {(*this).NoSpace()<<red_fg<<cyan_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_warning_t))     {(*this).NoSpace()<<green_fg<<yellow_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_error_t))       {(*this).NoSpace()<<normal_fg<<blue_bg; return (*this).Space();}
  inline DebugStream &operator<<(void(*)(debug_fatal_error_t)) {(*this).NoSpace()<<normal_fg<<red_bg; return (*this).Space();}

  inline DebugStream &operator<<(const char *str)         {(*this).printf("%s",str);return MayBeSpace();}
  inline DebugStream &operator<<(const std::string& str)  {(*this).printf("%s",str.c_str());return MayBeSpace();}
  inline DebugStream &operator<<(bool t)                  {(*this).printf(t?"true":"false");return MayBeSpace();}
  inline DebugStream &operator<<(char t)                  {(*this).printf("%c",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed short t)          {(*this).printf("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned short t)        {(*this).printf("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed int t)            {(*this).printf("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned int t)          {(*this).printf("%d",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed long t)           {(*this).printf("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned long t)         {(*this).printf("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(signed long long t)      {(*this).printf("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(unsigned long long t)    {(*this).printf("%ld",t);return MayBeSpace();}
  inline DebugStream &operator<<(float t)                 {(*this).printf("%f",t);return MayBeSpace();}
  inline DebugStream &operator<<(double t)                {(*this).printf("%lf",t);return MayBeSpace();}
  inline DebugStream &operator<<(const void * t)          {return *this;}
  inline DebugStream &operator<<(std::nullptr_t)          {(*this).printf("(nullptr)");return *this;}
  inline DebugStream &operator<<(const char16_t* t)       {(*this).printf("%s",t);return MayBeSpace();}
  inline DebugStream &operator<<(char16_t t)              {(*this).printf("%c",t);return MayBeSpace();}
  inline DebugStream &operator<<(char32_t t)              {(*this).printf("%c",t);return MayBeSpace();}

  //======================================
  template <typename T>
  DebugStream& operator<<(const T& value) {
    #if PPRINT_EN
    (*this).print(value);
    if(!pprint_stream.str().empty()) {
      (*this).NoNewLine() << pprint_stream.str();
      pprint_stream.str("");
    }
    #else
    static_cast<std::stringstream&>(*this) << value;
    if (!(*this).str().empty()) {
      (*this) << (*this).str();
      (*this).str("");
    }
    #endif

    return *this;
  }
  //======================================
 public:
  DebugStream &printf(const char *fmt, ...);
  inline DebugStream &operator()() {
     return *this;
  }
  inline DebugStream& operator()(const char* varName) {
    (*this).printf(varName);
    return *this;
  }
  template <typename T>
  inline DebugStream& operator()(const char* varName, const T& value) {
    (*this) << varName << "=" << value;
    return *this;
  }
  // inline DebugStream& operator()(const char* varName, const char* value) {
  //   (*this) << varName << "=" << value;
  //   return *this;
  // }
  template <typename... Args>
  inline DebugStream& operator()(const char* varName, const char* fmt, Args... args) {
    (*this).printf(fmt,args...);
    return *this;
  }
  //======================================
 private:
  inline DebugStream &MayBeSpace() {
    if (!newline_&&space) (*this)(" ");
    newline_ = false;
    return *this;
  }
  //======================================
 private:
  std::function<void(const char*,int)> fun;
  int buf_len{256};
  std::unique_ptr<char[]> buffer;
  bool out_en{true};
  bool space{true};
  bool newline{false};
  bool newline_{false}; // solve the bug that add newline still add space
  bool terminate{false}; // if true will terminate program if true Use for debug error info
  bool clear_color{false}; // if true will clear color when deconstruct object
#if PPRINT_EN
 public:
  std::stringstream pprint_stream;
#endif
};

inline DebugStream::DebugStream
(std::function<void(const char*,int)>  fun_ , int buf_len_ ) : fun(fun_) ,buf_len(buf_len_)
#if PPRINT_EN
, pprint::PrettyPrinter(pprint_stream)  
#endif
{
  buffer = std::unique_ptr<char[]>(new char[buf_len]);
}

inline DebugStream::DebugStream(const DebugStream &obj) {
  this->buf_len = obj.buf_len;
  this->out_en = obj.out_en;
  this->fun = obj.fun;
  this->buffer = std::unique_ptr<char[]>(new char[buf_len]);
}

inline DebugStream::DebugStream(DebugStream &&obj) {
  this->buf_len = obj.buf_len;
  this->out_en = obj.out_en;
  this->fun = obj.fun;
  buffer=std::move(obj.buffer);
}

inline DebugStream &DebugStream::operator=(const DebugStream &obj) {
  if (this != &obj) {
    this->buf_len = obj.buf_len;
    this->out_en = obj.out_en;
    this->fun = obj.fun;
    this->buffer = std::unique_ptr<char[]>(new char[buf_len]);
  }
  return *this;
}

inline DebugStream &DebugStream::operator=(DebugStream &&obj) {
  if (this != &obj) {
    this->buf_len = obj.buf_len;
    this->out_en = obj.out_en;
    this->fun = obj.fun;
    this->buffer = std::move(obj.buffer);
  }
  return *this;
}

inline DebugStream::~DebugStream() {
  if(terminate) std::terminate(); 
  if(buffer==nullptr) return; // If buffer is nullptr, then cannot use print
  if(clear_color) (*this)<<normal_fg<<normal_bg; 
  if(newline) (*this)("\n"); // send a "\n"
}

inline DebugStream &DebugStream::printf(const char *fmt, ...) {
#ifdef NO_DEBUG_OUTPUT //
  return *this;
#endif // NO_DEBUG_OUTPUT
  if (!this->out_en) {
    return *this;
  }
  va_list ap;
  va_start(ap, fmt);
  vsprintf((char *) buffer.get() , fmt, ap);
  va_end(ap);
  int i = strlen((const char *) buffer.get());
  fun(buffer.get(), i);

  // solve the bug that add newline still add space
  if(buffer.get()[i-1]==0X0A||buffer.get()[i-1]==0X0D) {
    newline_= true;
  }

  return *this;
}

// FILE_LINE
inline std::string FileLine(const std::string& file_name="",int line_num=-1) {
  std::string res;
  if(line_num<0) {
    res = file_name;
  } else if (file_name=="") {
    res = std::to_string(line_num);
  } else {
    res = file_name+":"+std::to_string(line_num);
  }
  return res;
}
#define FILE_LINE gxt::FileLine(__FILE__, __LINE__)
#define FILE gxt::FileLine(__FILE__, -1)
#define LINE gxt::FileLine("", __LINE__)


// Type Name Implement
template <typename T>
inline std::string TypeImpl() {
#ifdef _MSC_VER // msvc support
  std::string str=__FUNCSIG__;
  auto posi_start = str.find(",");
  posi_start += 1;
  auto posi_end=str.find_first_of(">",posi_start);
#else // gcc and clangd
  std::string str=__PRETTY_FUNCTION__;
  auto posi_start = str.find("T = ");
  posi_start += 4;
  auto posi_end=str.find_first_of(";]",posi_start);
#endif
  return str.substr(posi_start,posi_end-posi_start);
}
#define TYPET(type) (gxt::TypeImpl<type>())
#define TYPE(type) (gxt::TypeImpl<decltype(type)>())

// Usage: gDebug() << VAR(a) // stdout: a = ${a}
#define VAR(x) #x<<"="<<x 

// Define a macro to get the parameter at the specified position in the parameter list
#define GET_ARG(N, ...) GET_ARG_##N (__VA_ARGS__)
#define GET_ARG_1(arg, ...) arg
#define GET_ARG_2(arg, ...) GET_ARG_1(__VA_ARGS__) 
#define GET_ARG_3(arg, ...) GET_ARG_2(__VA_ARGS__) 
#define GET_ARG_4(arg, ...) GET_ARG_3(__VA_ARGS__) 
#define GET_ARG_5(arg, ...) GET_ARG_4(__VA_ARGS__) 


// Get the number of input parameters
template <typename ...T>
__attribute__((deprecated))
inline int GetParaNumber(T ...para){ return sizeof...(para); }

// Prevent input null parameter
template <typename T,T value>
__attribute__((deprecated))
inline T PreventNULL(const T& para){ return para;}
template <typename T,T value>
__attribute__((deprecated))
inline T PreventNULL(){return value;}

};

#define gDebugN(a,...) \
    [=](int cnt = 1,int max_print_cnt = -1) { \
      static int cnt_i{cnt}; \
      static int max_print_i{0}; \
      if(max_print_cnt>=0&&max_print_i>=max_print_cnt) return; \
      if (cnt_i++ >= cnt) { \
        gDebug(a); \
        cnt_i = 1; \
        ++max_print_i; \
      } \
    }(__VA_ARGS__)


// #define gDebug (gxt::DebugStream(gxt::DebugSendStringCallBack_Default_)<<gxt::normal_fg<<gxt::normal_bg)
#define gDebug(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::normal_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugWarn(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::black_fg<<gxt::yellow_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugError(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor().Terminate()<<gxt::white_fg<<gxt::red_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))


#define gDebugCol1(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::green_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugCol2(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::blue_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugCol3(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::magenta_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugCol4(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::cyan_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))
#define gDebugCol5(...) ((gxt::DebugStream(gxt::DebugSendStringCallBack_Default_).NewLine().ClearColor()<<gxt::red_fg<<gxt::normal_bg)(#__VA_ARGS__ __VA_OPT__(,) __VA_ARGS__))


#include <chrono>

// 定义宏 TIME_BEGIN 来开始计时
#define TIME_BEGIN(...) \
    auto __start_time__##__VA_ARGS__ = std::chrono::high_resolution_clock::now();

// 定义宏 TIME_END 来打印输出执行时间
#define TIME_END(...) \
    {\
      auto __end_time__##__VA_ARGS__ = std::chrono::high_resolution_clock::now(); \
      auto __duration_time__##__VA_ARGS__ = std::chrono::duration_cast<std::chrono::milliseconds>(__end_time__##__VA_ARGS__ - __start_time__##__VA_ARGS__).count(); \
      if(std::string(#__VA_ARGS__).empty()) \
      std::cout << "Default" << " Execution time: " << __duration_time__##__VA_ARGS__ << " ms" << std::endl; \
      else \
      std::cout << #__VA_ARGS__ << " Execution time: " << __duration_time__##__VA_ARGS__ << " ms" << std::endl; \
    }

#define TIME_LOOP(...) \
  { \
    static auto __time_loop_begin_##__VA_ARGS__ = std::chrono::high_resolution_clock::now(); \
    static size_t __time_loop_i__##__VA_ARGS__ = 0; \
    auto __time_loop_end_##__VA_ARGS__ = std::chrono::high_resolution_clock::now(); \
    auto __loop_duration_time__##__VA_ARGS__ = std::chrono::duration_cast<std::chrono::milliseconds>(__time_loop_end_##__VA_ARGS__ - __time_loop_begin_##__VA_ARGS__).count(); \
    __time_loop_begin_##__VA_ARGS__=__time_loop_end_##__VA_ARGS__;\
    if(__time_loop_i__##__VA_ARGS__==0) std::cout << "TIME_LOOP(" << #__VA_ARGS__ << "):" << __time_loop_i__##__VA_ARGS__ << " initialize" << std::endl; \
    else std::cout << "TIME_LOOP(" << #__VA_ARGS__ << "):" << __time_loop_i__##__VA_ARGS__ << " Execution time: " << __loop_duration_time__##__VA_ARGS__ << " ms" << std::endl; \
    ++__time_loop_i__##__VA_ARGS__; \
  }

#endif //DEBUGSTREAM_H__


