#include <pine/psl/algorithm.h>
#include <pine/psl/iostream.h>
#include <pine/psl/optional.h>
#include <pine/psl/variant.h>
#include <pine/psl/memory.h>
#include <pine/psl/string.h>
#include <pine/psl/array.h>
#include <pine/psl/math.h>
#include <pine/psl/map.h>

#include <pine/core/log.h>

#include <cstdio>

void (*pine::log_stream)(psl::string_view data) = +[](psl::string_view data) {
  printf("%s", psl::string(data).c_str());
};
void (*pine::warning_stream)(psl::string_view data) = +[](psl::string_view data) {
  printf("%s", psl::string(data).c_str());
};
void (*pine::fatal_stream)(psl::string_view data) = +[](psl::string_view data) {
  printf("%s", psl::string(data).c_str());
};

struct Emitter {
  void expect(psl::vector<psl::string> exps) {
    backup_received.clear();
    expectations = psl::move(exps);
  }

  void operator()(psl::string value) {
    backup_received.push_back(value);
    auto history = psl::space_by(backup_received, " ");
    if (expectations.size() == 0)
      pine::Fatal("Expect null, get ", value, "\nHistory(inclusive): ", history);
    if (expectations.front() != value)
      pine::Fatal("Expect ", expectations.front(), ", get ", value,
                  "\nHistory(inclusive): ", history);
    expectations.pop_front();
  }

private:
  psl::vector<psl::string> expectations;
  psl::vector<psl::string> backup_received;
};
static Emitter emitter{};

void test_vector() {
  auto xs = psl::vector<int>(2, 1);
  auto ys = psl::vector_of(10, 20, 30, 40);
  CHECK_EQ(xs.size(), 2);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 1);
  CHECK_EQ(ys.size(), 4);
  CHECK_EQ(ys[0], 10);
  CHECK_EQ(ys[1], 20);
  CHECK_EQ(ys[2], 30);
  CHECK_EQ(ys[3], 40);

  xs = psl::vector_of(1, 2, 3);

  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 2);
  CHECK_EQ(xs[2], 3);

  xs = psl::move(xs);
  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 2);
  CHECK_EQ(xs[2], 3);

  xs.assign_from(ys);
  CHECK_EQ(xs.size(), 4);
  CHECK_EQ(xs[0], 10);
  CHECK_EQ(xs[1], 20);
  CHECK_EQ(xs[2], 30);
  CHECK_EQ(xs[3], 40);

  xs = psl::vector_of(-1, -2, -3);
  xs = ys;
  CHECK_EQ(xs.size(), 4);
  CHECK_EQ(xs[0], 10);
  CHECK_EQ(xs[1], 20);
  CHECK_EQ(xs[2], 30);
  CHECK_EQ(xs[3], 40);

  xs.pop_back();
  xs.pop_back();
  CHECK_EQ(xs.size(), 2);
  CHECK_EQ(xs[0], 10);
  CHECK_EQ(xs[1], 20);

  xs.push_back(5);
  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 10);
  CHECK_EQ(xs[1], 20);
  CHECK_EQ(xs[2], 5);

  xs.push_front(6);
  CHECK_EQ(xs.size(), 4);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);
  CHECK_EQ(xs[2], 20);
  CHECK_EQ(xs[3], 5);

  xs.pop_back();
  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);
  CHECK_EQ(xs[2], 20);

  xs.emplace_back(1);
  CHECK_EQ(xs.size(), 4);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);
  CHECK_EQ(xs[2], 20);
  CHECK_EQ(xs[3], 1);

  xs.pop_back();
  xs.pop_back();
  CHECK_EQ(xs.size(), 2);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);

  xs.insert(xs.begin(), 1);
  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 6);
  CHECK_EQ(xs[2], 10);

  xs.insert(psl::next(xs.begin()), 2);
  CHECK_EQ(xs.size(), 4);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 2);
  CHECK_EQ(xs[2], 6);
  CHECK_EQ(xs[3], 10);
  CHECK_EQ(xs.front(), 1);
  CHECK_EQ(xs.back(), 10);
  CHECK_EQ(xs, psl::vector_of(1, 2, 6, 10));
  CHECK_NE(xs, psl::vector_of(1, 2, 6));
  CHECK_NE(xs, psl::vector_of(1, 2, 6, 10, 0));
  CHECK_NE(xs, psl::vector_of(1, 2, 6, 11));

  xs.erase(psl::next(xs.begin()));
  CHECK_EQ(xs.size(), 3);
  CHECK_EQ(xs[0], 1);
  CHECK_EQ(xs[1], 6);
  CHECK_EQ(xs[2], 10);

  xs.pop_front();
  CHECK_EQ(xs.size(), 2);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);

  xs.erase(xs.end());
  CHECK_EQ(xs.size(), 2);
  CHECK_EQ(xs[0], 6);
  CHECK_EQ(xs[1], 10);

  xs.erase(psl::prev(xs.end()));
  CHECK_EQ(xs.size(), 1);
  CHECK_EQ(xs[0], 6);

  {
    auto v = psl::vector<psl::vector<int>>{};
    v.push_back(psl::vector_of(1, 3));
    v.push_back(psl::vector_of(1, 7));
    v.push_back(psl::vector_of(4, 7));
  }

  {
    auto v = psl::vector<psl::vector<int>>{};
    v.push_back(psl::vector_of(1, 3));
    v.push_back(psl::vector_of(1, 7));
    v.push_back(psl::vector_of(4, 7));
    auto v1 = v;
    v1.push_back(psl::vector_of(6, 1, 24));
  }
  {
    auto v = psl::vector<psl::vector<int>>{};
    v.push_back(psl::vector_of(1, 3));
    v.push_back(psl::vector_of(1, 7));
    v.push_back(psl::vector_of(4, 7));
    auto v1 = v;
    v1.push_back(psl::vector_of(6, 1, 24));
    v = psl::move(v1);
  }
  {
    auto v = psl::vector<psl::shared_ptr<int>>{};
    v.push_back(psl::make_shared<int>(1));
    v.push_back(psl::make_shared<int>(3));
    v.push_back(psl::make_shared<int>(7));
    auto v2 = v;
  }
  {
    auto v = psl::vector<psl::Variant<psl::vector<int>, psl::vector<float>>>{};
    v.push_back(psl::vector_of(1, 3));
    v.push_back(psl::vector_of(1, 7));
    v.push_back(psl::vector_of(1, 8));
    auto v2 = v;
  }
}

struct MockA {
  MockA() {
    emitter("Ac");
  }
  virtual ~MockA() {
  }
  virtual int& f(bool choice, int& a, int& b) {
    return choice ? b : a;
  }

  const psl::string id = "A";
};
struct MockB {
  MockB() {
    emitter("Bc");
  }
  ~MockB() {
  }
  int& f(bool choice, int& a, int& b) {
    return choice ? a : b;
  }

  const psl::string id = "B";
};
struct MockA1 : MockA {
  MockA1() {
    emitter("A1c");
  }
  ~MockA1() {
  }
  int& f(bool choice, int& a, int& b) override {
    return choice ? b : a;
  }

  const psl::string id = "A1";
};

void test_string() {
  auto x = psl::string("Hello");
  CHECK_EQ(x.size(), 5);
  CHECK_EQ(x.subview(0), "Hello");
  CHECK_EQ(x.subview(0, 100), "Hello");
  CHECK_EQ(x.subview(0, 0), "");
  CHECK_EQ(x.substr(0), "Hello");
  CHECK_EQ(x.substr(0, 100), "Hello");
  CHECK_EQ(x.substr(0, 0), "");
  CHECK_NE(x.substr(0, 2), "H");
  CHECK_NE(x.subview(0, 2), "H");
  CHECK_NE(x.substr(0, 2), "Hel");
  CHECK_NE(x.subview(0, 2), "Hel");
  CHECK_NE(x.substr(0, 2), "HE");
  CHECK_NE(x.subview(0, 2), "HE");
  CHECK_EQ(psl::string("Hello", 3), "Hel");
  CHECK_EQ(psl::string(5, 'k'), "kkkkk");
  CHECK_EQ(x + " ", "Hello ");
  CHECK_EQ(" " + x, " Hello");
  CHECK_EQ(x + " World", "Hello World");
  CHECK_EQ("World " + x, "World Hello");
}

void test_variant() {
  emitter.expect(psl::vector_of<psl::string>("Ac", "Bc"));
  auto x = psl::Variant<MockA, MockB>{};
  CHECK(!x.is_valid());
  x = MockA();
  CHECK(x.is<MockA>());
  int a = 1, b = 2;
  auto& c = x.dispatch([&a, &b](auto&& x) -> decltype(auto) { return x.f(false, a, b); });
  CHECK_EQ(c, 1);
  c = 3;
  CHECK_EQ(a, 3);
  x = MockB();
  auto& d = x.dispatch([&a, &b](auto&& x) -> decltype(auto) { return x.f(false, a, b); });
  CHECK_EQ(d, 2);
  d = 5;
  CHECK_EQ(b, 5);
  {
    auto a = psl::Variant<psl::vector<int>, psl::vector<float>>();
    auto c = psl::Variant<psl::vector<int>, psl::vector<float>>(psl::vector_of(3.0f, 1.0f, 7.0f));
    auto b = a;
    a = psl::vector_of(1, 3);
    b = c;
  }
  {
    auto a = psl::Variant<psl::Box<int>, psl::Box<float>>();
    auto c = psl::Variant<psl::Box<int>, psl::Box<float>>(psl::Box(7.0f));
    {
      auto b = a;
      a = psl::Box(1);
      b = c;
    }
  }
}

template <typename T>
struct MockDeleter {
  template <typename U>
  using ChangeBasis = MockDeleter<U>;

  MockDeleter() = default;
  template <typename U>
  requires psl::is_convertible<U*, T*>
  MockDeleter(const MockDeleter<U>&) {
  }

  void operator()(T* ptr) const {
    emitter(ptr->id);
    delete ptr;
  }
};

void test_memory() {
  {
    auto x = 5.0f;
    CHECK_EQ(psl::bitcast<float>(psl::bitcast<int>(x)), x);
  }
  {
    emitter.expect(psl::vector_of<psl::string>("Ac", "Bc", "B", "A"));
    auto a = psl::unique_ptr<const MockA, MockDeleter<const MockA>>(new MockA{});
    auto b = psl::unique_ptr<MockB, MockDeleter<MockB>>(new MockB{});
  }
  {
    emitter.expect(psl::vector_of<psl::string>("Ac", "Bc", "Ac", "A1c", "A", "B", "A"));
    auto a = psl::unique_ptr<const MockA, MockDeleter<const MockA>>(new MockA{});
    auto b = psl::unique_ptr<MockB, MockDeleter<MockB>>(new MockB{});
    a = psl::unique_ptr<MockA1, MockDeleter<MockA1>>(new MockA1{});
  }
  {
    emitter.expect(psl::vector_of<psl::string>("Ac", "Bc", "B", "A"));
    auto a = psl::shared_ptr<const MockA, MockDeleter<const MockA>>(new MockA{});
    auto b = psl::shared_ptr<MockB, MockDeleter<MockB>>(new MockB{});
  }
  {
    emitter.expect(psl::vector_of<psl::string>("Ac", "Bc", "Ac", "A1c", "B", "A", "A"));
    auto a = psl::shared_ptr<const MockA, MockDeleter<const MockA>>(new MockA{});
    auto c = a;
    a = c;
    auto d = a;
    { auto e = c; }
    auto b = psl::shared_ptr<MockB, MockDeleter<MockB>>(new MockB{});
    a = psl::shared_ptr<MockA1, MockDeleter<MockA1>>(new MockA1{});
  }
  {
    auto ov = psl::make_unique<psl::vector<int>>();
    *ov = psl::vector_of(1, 3, 7);
    ov->push_back(3);
    auto ov4 = psl::make_unique<psl::vector<int>>(psl::vector_of(1, 2));
    auto ov1 = psl::move(ov);
    ov1->push_back(7);
    ov1->push_back(7);
  }
  {
    auto ov = psl::make_shared<psl::vector<int>>();
    *ov = psl::vector_of(1, 3, 7);
    auto ov1 = ov;
    ov->push_back(3);
    auto ov4 = ov;
    ov->push_back(7);
    {
      auto ov2 = ov1;
      auto ov3 = psl::move(ov1);
    }
    ov4 = psl::make_shared<psl::vector<int>>(psl::vector_of(1, 2));
  }
  {
    auto x = 10;
    auto y = psl::ref{x};
    y = 20;
    CHECK_EQ(y, 20);
    CHECK_EQ(x, 20);
    auto z = y;
    z = 30;
    CHECK_EQ(x, 30);
    CHECK_EQ(y, 30);
  }
  {
    auto x = psl::Box{1};
    *x = 2;
    auto y = x;
    CHECK_EQ(*y, 2);
    *y = 3;
    CHECK_EQ(*x, 2);
    CHECK_EQ(*y, 3);
  }
  {
    auto ov = psl::Box<psl::vector<int>>(psl::vector_of(1, 3, 7));
    auto ov1 = ov;
    ov->push_back(3);
    ov->push_back(7);
    auto ov4 = ov;
    {
      auto ov2 = ov1;
      auto ov3 = psl::move(ov1);
    }
    ov4 = psl::Box<psl::vector<int>>{psl::vector_of(1, 2)};
  }
}

void test_optional() {
  struct Book {
    Book(int pages) : pages{pages} {
      emitter("BookC");
    }

    int pages;
  };

  emitter.expect(psl::vector_of<psl::string>("BookC", "BookC"));
  auto book = psl::optional<Book>{psl::nullopt};
  CHECK(!book);
  book = Book(347);
  CHECK(book);
  CHECK_EQ(book->pages, 347);
  { auto book2 = book; }
  CHECK_EQ(book->pages, 347);
  {
    auto book2 = psl::optional<Book>{Book(100)};
    book = book2;
  }
  CHECK_EQ(book->pages, 100);

  auto ov = psl::optional<psl::vector<int>>{};
  ov = psl::vector_of(1, 3, 7);
  auto ov1 = ov;
  ov->push_back(3);
  ov->push_back(7);
  {
    auto ov2 = ov1;
    auto ov4 = ov1;
    auto ov3 = psl::move(ov1);
    ov4 = psl::optional<psl::vector<int>>{psl::vector_of(1, 2)};
  }
}

void test_array() {
  auto xs = psl::Array{1, 5, 3};
  auto ys = xs;
  CHECK_EQ(ys[0], 1);
  CHECK_EQ(ys[1], 5);
  CHECK_EQ(ys[2], 3);
}

void test_algorithm() {
  {
    auto xs = psl::Array{1, 2, 3};
    auto ys = psl::Array{10, 20, 30};
    auto first = psl::begin(xs);
    auto last = psl::end(xs);
    CHECK_EQ(psl::distance(first, last), 3);

    psl::copy(first, ys);
    CHECK_EQ(xs, ys);

    xs = {1, 2, 3};
    psl::move(first, ys);
    CHECK_EQ(xs, ys);

    auto it = psl::lower_bound(xs, psl::less_than{15});
    CHECK(it != xs.end());
    CHECK_EQ(*it, 20);

    it = psl::lower_bound(xs, psl::less_than{10});
    CHECK(it != xs.end());
    CHECK_EQ(*it, 10);

    it = psl::lower_bound(xs, psl::less_than{-10});
    CHECK(it != xs.end());
    CHECK_EQ(*it, 10);

    it = psl::lower_bound(xs, psl::less_than{40});
    CHECK(it == xs.end());

    it = psl::find(xs, 20);
    CHECK(it != xs.end());
    CHECK_EQ(*it, 20);

    it = psl::find(xs, 21);
    CHECK(it == xs.end());
  }
  {
    auto xs = psl::Array{1, 3, 2, 2, 4};
    auto ys = xs;

    auto it = psl::find_last_of(xs, 2);
    CHECK(it == psl::prev(xs.end(), 2));

    psl::replace(xs, 3, 5);
    CHECK_EQ(xs, psl::Array(1, 5, 2, 2, 4));
    CHECK_EQ(psl::count(xs, 2), 2);

    psl::sort(xs, psl::less<>{});
    CHECK_EQ(xs, psl::Array(1, 2, 2, 4, 5));

    xs = ys;
    it = psl::partition(xs, psl::less_than{3});
    CHECK(it == psl::prev(xs.end(), 2));
    CHECK_LT(psl::max(xs[0], xs[1], xs[2]), 3);
    CHECK_GE(psl::min(xs[3], xs[4]), 3);

    xs = ys;
    it = psl::partition(xs, psl::less_than{2});
    CHECK(it == psl::next(xs.begin()));
    CHECK_LT(xs[0], 2);

    xs = ys;
    it = psl::partition(xs, psl::less_than{3});
    CHECK(it == psl::next(xs.begin(), 3));
    CHECK_EQ(xs[4], 4);

    xs = ys;
    psl::nth_element(xs, psl::next(xs.begin()), psl::less<>{});
    // CHECK(it == psl::prev(xs.end(), 2));

    xs = ys;
    psl::nth_element(xs, psl::next(xs.begin(), 2), psl::less<>{});
    // CHECK(it == psl::next(xs.begin()) || it == psl::next(xs.begin(), 2));

    xs = ys;
    psl::partial_sort(xs, psl::next(xs.begin()), psl::less<>{});
    CHECK_EQ(xs[0], 1);
    CHECK_EQ(xs[1], 2);

    xs = ys;
    psl::partial_sort(xs, psl::next(xs.begin(), 3), psl::less<>{});
    CHECK_EQ(xs[0], 1);
    CHECK_EQ(xs[1], 2);
    CHECK_EQ(xs[2], 2);
    CHECK_EQ(xs[3], 3);

    xs = ys;
    psl::reverse(xs);
    CHECK_EQ(xs, psl::Array(4, 2, 2, 3, 1));
    auto zs = to<psl::vector<int>>(psl::transform(xs, [](int x) { return x * 2; }));
    CHECK_EQ(zs, psl::vector_of(8, 4, 4, 6, 2));
  }
}

int main() {
  test_vector();
  test_variant();
  test_string();
  test_memory();
  test_optional();
  test_algorithm();
}