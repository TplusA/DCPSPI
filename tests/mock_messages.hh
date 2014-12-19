#ifndef MOCK_MESSAGES_HH
#define MOCK_MESSAGES_HH

#include "messages.h"
#include "mock_expectation.hh"

class MockMessages
{
  private:
    MockMessages(const MockMessages &);
    MockMessages &operator=(const MockMessages &);

  public:
    class Expectation;
    typedef MockExpectationsTemplate<Expectation> MockExpectations;
    MockExpectations *expectations_;

    bool ignore_all_;

    explicit MockMessages();
    ~MockMessages();

    void init();
    void check() const;

    void expect_msg_error_formatted(int error_code, int priority, const char *string);
    void expect_msg_error_formatted(int error_code, int priority, const char *prefix, const char *suffix);
    void expect_msg_error(int error_code, int priority, const char *string);
    void expect_msg_info_formatted(const char *string);
    void expect_msg_info(const char *string);
};

/*!
 * One messages mock to rule them all...
 *
 * This is necessary because there are only free C function tested by this
 * mock, and there is no simple way for these functions to pick a suitable mock
 * object from a set of those. It should be possible to use TLD for this
 * purpose, but for now let's go with a simpler version.
 *
 * \note Having this singleton around means that running tests in multiple
 *     threads in NOT possible.
 */
extern MockMessages *mock_messages_singleton;

#endif /* !MOCK_MESSAGES_HH */
