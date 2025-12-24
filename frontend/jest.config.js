// frontend/jest.config.js
module.exports = {
  testEnvironment: 'jest-environment-jsdom',
  setupFilesAfterEnv: ['<rootDir>/setupTests.js'],
  transform: {
    '^.+\.(js|jsx|ts|tsx)$': 'babel-jest',
  },
};
