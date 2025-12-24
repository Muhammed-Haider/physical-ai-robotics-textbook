// frontend/tests/App.test.js
import React from 'react';
import { render, screen } from '@testing-library/react';

test('renders learn react link', () => {
  render(<h1>Hello, Docusaurus!</h1>);
  const linkElement = screen.getByText(/Hello, Docusaurus!/i);
  expect(linkElement).toBeInTheDocument();
});
