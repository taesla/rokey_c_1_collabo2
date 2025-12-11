// src/api/people.ts
import { api } from "./client";

export type Person = {
  id: number;
  military_serial: string;
  name: string;
  department: string;
  rank?: string | null;
  created_at: string;
};

export async function fetchPeople(): Promise<Person[]> {
  const res = await api.get<Person[]>("/people/");
  return res.data;
}

export type CreatePersonPayload = {
  military_serial: string;
  name: string;
  department: string;
  rank?: string;
};

export async function createPerson(data: FormData) {
  return api.post("/people/register", data, {
    headers: { "Content-Type": "multipart/form-data" },
  });
}


export async function deletePerson(serial: string) {
  return api.delete(`/people/${serial}`);
}



export async function searchPerson(serial: string) {
    const res = await api.get(`/people/search/${serial}`);
    return res.data;
}
