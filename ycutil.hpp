#pragma once
#include <optional>
#include <string>

template <class ContainerType>
concept Container = requires(ContainerType a, const ContainerType b)
{
	requires std::regular<ContainerType>;
	requires std::swappable<ContainerType>;
	requires std::destructible<typename ContainerType::value_type>;
	requires std::same_as<typename ContainerType::reference, typename ContainerType::value_type&>;
	requires std::same_as<typename ContainerType::const_reference, const typename ContainerType::value_type&>;
	requires std::forward_iterator<typename ContainerType::iterator>;
	requires std::forward_iterator<typename ContainerType::const_iterator>;
	requires std::signed_integral<typename ContainerType::difference_type>;
	requires std::same_as<typename ContainerType::difference_type, typename std::iterator_traits<typename ContainerType::iterator>::difference_type>;
	requires std::same_as<typename ContainerType::difference_type, typename std::iterator_traits<typename ContainerType::const_iterator>::difference_type>;
	{ a.begin() }	-> std::convertible_to<typename ContainerType::iterator>;
	{ a.end() }		-> std::convertible_to<typename ContainerType::iterator>;
	{ b.begin() }	-> std::convertible_to<typename ContainerType::const_iterator>;
	{ b.end() }		-> std::convertible_to<typename ContainerType::const_iterator>;
	{ a.cbegin() }	-> std::convertible_to<typename ContainerType::const_iterator>;
	{ a.cend() }	-> std::convertible_to<typename ContainerType::const_iterator>;
	{ a.size() }	-> std::convertible_to<typename ContainerType::size_type>;
	{ a.max_size() }-> std::convertible_to<typename ContainerType::size_type>;
	{ a.empty() }	-> std::convertible_to<bool>;
};

namespace yc {
	template <typename T>
	struct err_opt_t : public std::optional<T> {
		std::string err;
		
		err_opt_t() {}
		explicit err_opt_t(T& value) { *static_cast<std::optional<T>*>(this) = value; }
		explicit err_opt_t(T&& value) { *static_cast<std::optional<T>*>(this) = std::move(value); }
		err_opt_t(err_opt_t<T>& other) { operator=(other); }
		err_opt_t(err_opt_t<T>&& other) noexcept { operator=(other); }
		
		err_opt_t(const char* e) { err = e;}
		err_opt_t(const std::string e) { err = e; }
		err_opt_t(const std::string&& e) { err = e; }
		err_opt_t& operator= (const std::string&& e) { err = e; return *this; }
		err_opt_t& operator= (const char* e) { err = e; return *this; }
		
		err_opt_t& operator= (err_opt_t<T>& other) {
			if (other.has_value()) *static_cast<std::optional<T>*>(this) = other.value();
			else													 err = other.err;
			return *this;
		}
		err_opt_t& operator= (err_opt_t<T>&& other) noexcept {
			if (other.has_value()) *static_cast<std::optional<T>*>(this) = other.value();
			else													 err = other.err;
			return *this;
		}
		err_opt_t& operator= (T& value) {
			*static_cast<std::optional<T>*>(this) = value;
			return *this;
		}
		err_opt_t& operator= (T&& value) {
			*static_cast<std::optional<T>*>(this) = std::move(value);
			return *this;
		}

		auto operator| (auto&& function) -> decltype(function(this->value())) {
			if (this->has_value()) return function (this->value());
			return err;
		}
		
		auto operator| (auto&& function) -> decltype(function()) {
			if (this->has_value()) return function();
			return err;
		}

	};
}
