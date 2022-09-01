#ifndef XRSLAM_TAGGED_H
#define XRSLAM_TAGGED_H

#include <xrslam/common.h>

namespace xrslam {

template <typename TagEnum,
          typename = std::enable_if_t<std::is_enum_v<std::decay_t<TagEnum>>>>
struct Tagged {
  private:
    template <TagEnum V>
    [[nodiscard]] static constexpr std::string_view enum_value_name() noexcept {
#if defined(__clang__)
        constexpr std::string_view name{__PRETTY_FUNCTION__,
                                        sizeof(__PRETTY_FUNCTION__) - 2};
#elif defined(__GNUC__) && __GNUC__ >= 9
        constexpr std::string_view name{__PRETTY_FUNCTION__,
                                        sizeof(__PRETTY_FUNCTION__) - 51};
#elif defined(_MSC_VER)
        constexpr std::string_view name{__FUNCSIG__, sizeof(__FUNCSIG__) - 17};
#else
#error "Unsupported compiler."
        return {}; // Unsupported compiler.
#endif
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 9) ||              \
    defined(_MSC_VER)
        constexpr auto prefix = name.find_last_of(" :,-)") + 1;
        if constexpr (name[prefix] >= '0' && name[prefix] <= '9') {
            return {}; // Value does not have name.
        } else {
            return name.substr(prefix, name.length() - prefix);
        }
#endif
    }

    template <int... I>
    [[nodiscard]] static constexpr auto
    enum_value_count(std::integer_sequence<int, I...>) noexcept {
        constexpr int n = sizeof...(I);
        constexpr std::array<bool, n> valid{
            {!enum_value_name<static_cast<TagEnum>(I)>().empty()...}};
        constexpr int num_valid = ((valid[I] ? 1 : 0) + ...);
        return num_valid;
    }

    [[nodiscard]] static constexpr size_t enum_size() noexcept {
        constexpr auto all_possible_tags =
            std::make_integer_sequence<int, 128>{};
        return enum_value_count(all_possible_tags);
    }

    static constexpr size_t tag_enum_size = enum_size();
    using bitset = std::bitset<tag_enum_size>;

    bitset tags;

  public:
    [[nodiscard]] static constexpr size_t tag_count() noexcept {
        return tag_enum_size;
    }

    Tagged() { tags.reset(); }

    bool tag(TagEnum f) const { return tags[static_cast<size_t>(f)]; }

    typename bitset::reference tag(TagEnum f) {
        return tags[static_cast<size_t>(f)];
    }

    template <typename... U> bool any_tagged(U... u) {
        std::initializer_list<TagEnum> tags{u...};
        return std::any_of(tags.begin(), tags.end(),
                           [this](TagEnum f) { return tag(f); });
    }

    template <typename... U> bool all_tagged(U... u) {
        std::initializer_list<TagEnum> tags{u...};
        return std::all_of(tags.begin(), tags.end(),
                           [this](TagEnum f) { return tag(f); });
    }

    template <typename... U> bool none_tagged(U... u) {
        std::initializer_list<TagEnum> tags{u...};
        return std::none_of(tags.begin(), tags.end(),
                            [this](TagEnum f) { return tag(f); });
    }

  protected:
    Tagged(const Tagged &tagged) : tags(tagged.tags) {}
};

} // namespace xrslam

#endif // XRSLAM_TAGGED_H
