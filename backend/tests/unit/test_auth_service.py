import pytest
from backend.src.services.auth_service import AuthService

@pytest.fixture
def auth_service():
    return AuthService()

async def test_authenticate_user_success(auth_service):
    # Test successful authentication for admin
    admin_user = await auth_service.authenticate_user("admin", "admin")
    assert admin_user is not None
    assert admin_user["username"] == "admin"
    assert admin_user["role"] == "admin"

    # Test successful authentication for regular user
    regular_user = await auth_service.authenticate_user("user", "user")
    assert regular_user is not None
    assert regular_user["username"] == "user"
    assert regular_user["role"] == "end-user"

async def test_authenticate_user_failure(auth_service):
    # Test authentication with incorrect password
    invalid_user = await auth_service.authenticate_user("admin", "wrong_password")
    assert invalid_user is None

    # Test authentication with non-existent user
    non_existent_user = await auth_service.authenticate_user("non_existent", "password")
    assert non_existent_user is None

async def test_create_access_token(auth_service):
    token = await auth_service.create_access_token("test_user_id", "test_role")
    assert token is not None
    assert "mock_access_token" in token
    assert "test_user_id" in token
    assert "test_role" in token

async def test_verify_token_success(auth_service):
    # Test verification for admin token
    admin_token = await auth_service.create_access_token("admin_uuid", "admin")
    verified_admin = await auth_service.verify_token(admin_token)
    assert verified_admin is not None
    assert verified_admin["user_id"] == "admin_uuid"
    assert verified_admin["role"] == "admin"

    # Test verification for regular user token
    user_token = await auth_service.create_access_token("user_uuid", "end-user")
    verified_user = await auth_service.verify_token(user_token)
    assert verified_user is not None
    assert verified_user["user_id"] == "user_uuid"
    assert verified_user["role"] == "end-user"

async def test_verify_token_failure(auth_service):
    # Test verification of an invalid token
    invalid_token = await auth_service.verify_token("invalid_mock_token")
    assert invalid_token is None

    # Test verification of a token for a non-existent user/role combination in mock
    non_mock_token = await auth_service.create_access_token("unknown_id", "unknown_role")
    unverified_mock = await auth_service.verify_token(non_mock_token)
    assert unverified_mock is None